-- LOVIS Truck Telemetry auth setup
-- Run this in Supabase SQL Editor after creating the project.

do $$
begin
  create type public.app_role as enum ('driver', 'tech', 'admin');
exception
  when duplicate_object then null;
end;
$$;

create table if not exists public.profiles (
  id uuid primary key references auth.users(id) on delete cascade,
  email text not null,
  role public.app_role not null default 'driver',
  created_at timestamptz not null default now(),
  updated_at timestamptz not null default now()
);

alter table public.profiles enable row level security;

drop policy if exists "Users can read their own profile" on public.profiles;

create policy "Users can read their own profile"
on public.profiles
for select
to authenticated
using (auth.uid() = id);

-- Keep role changes off the public client. For now, promote users manually in SQL.
-- The later admin portal should update roles through a protected server function.

create or replace function public.handle_new_user()
returns trigger
language plpgsql
security definer
set search_path = public
as $$
begin
  insert into public.profiles (id, email, role)
  values (new.id, coalesce(new.email, ''), 'driver')
  on conflict (id) do nothing;
  return new;
end;
$$;

drop trigger if exists on_auth_user_created on auth.users;

create trigger on_auth_user_created
after insert on auth.users
for each row execute function public.handle_new_user();

-- Promote a user after they have signed up or been invited:
-- update public.profiles set role = 'tech' where email = 'tech@example.com';
-- update public.profiles set role = 'admin' where email = 'owner@example.com';

-- If the user existed before this trigger was installed, create/promote in one step:
-- insert into public.profiles (id, email, role)
-- select id, email, 'tech'::public.app_role from auth.users where email = 'tech@example.com'
-- on conflict (id) do update set role = excluded.role, updated_at = now();

-- Server-side telemetry cache.
-- Netlify Functions write/read this with the Supabase service role key.
-- Do not add anon/authenticated browser policies here unless you also enforce
-- driver-vs-tech field filtering in SQL. The production site should read this
-- through /.netlify/functions/telemetry.

create table if not exists public.truck_telemetry_cache (
  truck_id text primary key default 'default',
  latest jsonb not null default '{}'::jsonb,
  rows jsonb not null default '[]'::jsonb,
  row_count integer not null default 0,
  source_updated_at timestamptz,
  fetched_at timestamptz not null default now(),
  source text not null default 'google_sheets',
  refresh_error text,
  updated_at timestamptz not null default now()
);

alter table public.truck_telemetry_cache enable row level security;

-- Keep the table locked to browser clients. The service role used by the
-- Netlify Function can still perform server-side administrative reads/writes.
drop policy if exists "No direct browser telemetry cache access" on public.truck_telemetry_cache;

create or replace function public.touch_updated_at()
returns trigger
language plpgsql
as $$
begin
  new.updated_at = now();
  return new;
end;
$$;

drop trigger if exists touch_truck_telemetry_cache_updated_at on public.truck_telemetry_cache;

create trigger touch_truck_telemetry_cache_updated_at
before update on public.truck_telemetry_cache
for each row execute function public.touch_updated_at();
