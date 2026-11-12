# Security Policy

## Deployment Model
`esp32-s3-dsp` is designed strictly as a **private, LAN-only** audio processor appliance:
- No external cloud services, external CDNs, or outbound internet telemetry are used.
- DSP tuning and input selection are intentionally kept open on the local network for zero-friction control.
- Administrative actions (`/update`, `/api/reboot`, `/console`, `/api/logs`, `/api/auth`, `/api/wifi` in STA mode, `/api/resampler/*`) are protected by HTTP Digest authentication (default `admin` / `admin`) and same-origin `Origin` header validation.

## Recommendations for Users
1. **Change the default password** via `POST /api/auth` before placing the device on a shared or untrusted Wi-Fi network.
2. **Do not expose** the device directly to the public internet (no port forwarding).
3. Keep `src/wifi_credentials.h` local only; never commit it to version control.

## Reporting a Vulnerability
Please open a private security advisory on GitHub or contact the repository maintainer before publicly disclosing vulnerabilities affecting OTA, authentication, or network parsing.
