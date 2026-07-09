# LOVIS Truck Telemetry Production Setup

## What Changes For Production

The browser no longer reads Google Sheets directly. It calls:

```text
/.netlify/functions/telemetry
```

That Netlify Function verifies the Supabase login, checks the user's role, refreshes Google Sheets only when needed, stores the newest good payload in Supabase, and returns role-filtered telemetry.

## Supabase

1. Open Supabase SQL Editor.
2. Run `supabase_auth_setup.sql`.
3. Invite users in Supabase Auth.
4. Promote technicians manually until the admin portal exists:

```sql
update public.profiles
set role = 'tech', updated_at = now()
where email = 'tech@example.com';
```

The `truck_telemetry_cache` table is intentionally locked down for browser users. The Netlify Function accesses it with the service role key.

## Netlify Site Settings

If this folder is used as the Netlify base folder:

```text
Base directory: outputs/dashboard2
Publish directory: .
Functions directory: netlify/functions
```

If the repo only contains this folder at the root, leave the base directory blank.

## Netlify Environment Variables

Set these in Netlify Site configuration > Environment variables:

```text
SUPABASE_URL=https://qnglgswbnqwumfmhmzcq.supabase.co
SUPABASE_PUBLISHABLE_KEY=your_supabase_publishable_key
SUPABASE_SERVICE_ROLE_KEY=your_supabase_secret_service_role_key
GOOGLE_SHEETS_API_URL=https://script.google.com/macros/s/AKfycbwnEGUfpvk-a3wJgBwrIRM6s1DaAasQKaMLhyy6VEVmq5pvRBiERz3GNtwLoKmrr3K8/exec
TELEMETRY_TRUCK_ID=default
TELEMETRY_NORMAL_REFRESH_MS=55000
TELEMETRY_SUPPORT_REFRESH_MS=15000
TELEMETRY_MAX_ROWS=2000
```

Never put `SUPABASE_SERVICE_ROLE_KEY` in the HTML or any client-side JavaScript.

## How Last-Good Storage Works

1. User logs in with Supabase.
2. Dashboard sends the user's access token to the Netlify Function.
3. Function verifies the token with Supabase Auth.
4. Function loads the user's role from `public.profiles`.
5. Function refreshes Google Sheets if the server cache is stale.
6. Function upserts the payload into `public.truck_telemetry_cache`.
7. Function returns driver-safe fields for drivers and all cached fields for tech/admin users.

If Google Sheets fails, the function still returns the last good Supabase cache when one exists.
