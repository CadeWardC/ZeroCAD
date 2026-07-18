# Release code signing

The release workflow (`.github/workflows/release.yml`) signs binaries
automatically when the secrets below exist in the GitHub repository settings
(Settings → Secrets and variables → Actions). With no secrets configured, every
signing step is skipped and the release ships unsigned — the pipeline never
fails because signing is unavailable.

## macOS — Developer ID + notarization

Requires an Apple Developer Program membership (US$99/year). This is the only
way to remove the Gatekeeper "unidentified developer" block; there is no free
path.

1. Enroll at <https://developer.apple.com/programs/>.
2. In Xcode or at <https://developer.apple.com/account/resources/certificates>,
   create a **Developer ID Application** certificate and export it (with its
   private key) as a `.p12` file with a password.
3. Create an **app-specific password** for your Apple ID at
   <https://account.apple.com> → Sign-In and Security (used by `notarytool`).
4. Add the secrets:

| Secret | Value |
| --- | --- |
| `MACOS_CERT_P12` | the `.p12` file, base64-encoded (`base64 -i cert.p12`) |
| `MACOS_CERT_PASSWORD` | the `.p12` export password |
| `APPLE_ID` | your Apple ID email |
| `APPLE_TEAM_ID` | 10-character team ID from the developer account page |
| `APPLE_APP_PASSWORD` | the app-specific password |

Result: the `.app` is signed with hardened runtime, the `.dmg` is notarized and
stapled — it opens on any Mac with no warnings.

## Windows — Authenticode

The workflow is wired for **Azure Trusted Signing** (~US$9.99/month, identity
validation available for individuals as well as organizations). Its
Microsoft-issued certificates build SmartScreen reputation far faster than
traditional OV certificates.

1. Create the resource: Azure Portal → *Trusted Signing Accounts* → create an
   account, complete identity validation, then create a certificate profile
   (Public Trust).
2. Create a Microsoft Entra **app registration** with a client secret, and give
   it the *Trusted Signing Certificate Profile Signer* role on the account.
3. Add the secrets:

| Secret | Value |
| --- | --- |
| `AZURE_TENANT_ID` | Entra tenant ID |
| `AZURE_CLIENT_ID` | app registration client ID |
| `AZURE_CLIENT_SECRET` | app registration client secret |
| `AZURE_TRUSTED_SIGNING_ENDPOINT` | e.g. `https://eus.codesigning.azure.net` |
| `AZURE_TRUSTED_SIGNING_ACCOUNT` | Trusted Signing account name |
| `AZURE_CERT_PROFILE_NAME` | certificate profile name |

Free alternative for open-source projects: [SignPath.io](https://signpath.io)
sponsors OSS with free signing (publisher shows as "SignPath Foundation"). It
uses a different CI integration (submit → sign → download via their GitHub
action), so the workflow's Windows signing step would need to be swapped out —
ask for this if the Azure cost isn't worth it.

Note: even with a valid signature, SmartScreen may briefly warn until the
certificate accumulates download reputation. This clears on its own; EV-level
validation and Trusted Signing shorten it.

## Linux

No equivalent concept — Linux has no Gatekeeper/SmartScreen, so the AppImage
runs without publisher warnings. Optional GPG signing (for users who verify
checksums) can be added later; checksums are already implied by the GitHub
release asset digests.
