# Offline Activation

Offline activation unlocks Serial Studio Pro on a machine that never connects to
the internet. Instead of validating the license online, the machine imports a
signed license file that you obtain once from the activation website.

This is meant for air-gapped, industrial, or locked-down deployments where the
computer running Serial Studio cannot reach the internet. A machine that has
occasional internet access does not need this: normal
[online activation](Pro-vs-Free.md#upgrading-from-free-to-pro) covers brief
disconnections with a 30-day offline grace period.

Offline activation is available only for lifetime and test-stand commercial
licenses. Monthly and yearly subscriptions use online activation with the
30-day offline grace period instead.

> **Warning:** activating an offline device permanently consumes one license
> seat. An air-gapped machine cannot report a deactivation, so there is no way
> to verify that a seat was released, and the seat cannot be freed or moved to
> another machine. Plan your seats before activating.

## What you need

- A Serial Studio Pro lifetime or test-stand license key and the email address
  used to purchase it.
- The offline machine, running a Pro build of Serial Studio.
- A second computer with internet access (a phone, laptop, or any browser).
- A way to move a small file between the two machines, such as a USB drive.

## Steps

Offline activation is a three-step exchange: the offline machine produces a
device file, the website turns it into a license file, and the offline machine
imports that license file.

### 1. Save the device file

On the offline machine:

1. Open the **About** dialog from the toolbar, then click **Manage License**.
2. In the Licensing dialog, click **Activate Offline…** to open the offline
   activation wizard.
3. Click **Save Device File…** and store the `.ssmachine` file.

The device file identifies this machine and carries no personal information. It
holds a hardware fingerprint and the application version.

### 2. Get the license file

On a computer with internet access:

1. Go to
   [serial-studio.com/offline-activation](https://serial-studio.com/offline-activation).
   The wizard also shows this address so you can type it on the other machine.
2. Upload the `.ssmachine` device file.
3. Enter the email address and license key from your purchase.
4. Download the `.sslic` license file the site returns.

The site verifies your license before it issues the file, so a valid Pro license
is required.

### 3. Import the license file

Move the `.sslic` file to the offline machine, then:

1. Reopen the wizard from **About → Manage License → Activate Offline…**.
2. Click **Import License File…** and select the `.sslic` file.

Pro features unlock immediately, and no network request is made on this machine.

## How the license behaves

- The license file is bound to the machine that produced the device file. It
  will not activate any other machine.
- Each offline activation permanently consumes one seat on your license.
  Unlike an online activation, the seat is never released: the machine cannot
  report a deactivation, and the server cannot verify one.
- The same machine can be reactivated by importing its `.sslic` file again, as
  long as its machine ID has not changed. The machine ID is derived from the
  operating system, not Serial Studio, and may change after an OS reinstall or
  a hardware replacement. Keep the license file somewhere safe.
- The certificate carries an expiry date set by the activation site, and
  Serial Studio checks it on every startup, including lifetime and test-stand
  licenses. When fewer than 14 days remain, the Licensing dialog shows a
  countdown and a warning to request a new certificate. Repeat the
  three-step exchange above to obtain and import a fresh `.sslic` file
  before the current one expires.
- The activation persists across restarts and needs no network on the offline
  machine.

## Managing seats

An offline seat is consumed permanently. The offline machine cannot reach the
internet, so it cannot report a deactivation, and the license server has no way
to verify that a seat was released on an air-gapped machine. Removing
the license from within Serial Studio only deletes the local certificate; the
seat stays in use.

Deleting the certificate is reversible on the same machine: import the `.sslic`
file again and Pro features unlock, provided the machine ID has not changed.
That ID depends on the operating system, so an OS reinstall or a hardware
replacement can invalidate it through no action of Serial Studio.

If an offline machine is destroyed or decommissioned, contact
alex@serial-studio.com.

## Troubleshooting

- **"This certificate was issued for a different device."** The license file was
  generated from another machine's device file. Export a new device file on the
  machine you are activating, then repeat the steps.
- **The import reports a malformed file.** Confirm you selected the `.sslic` file
  returned by the activation site, not the `.ssmachine` device file.

## See Also

- [Pro vs Free](Pro-vs-Free.md) - Feature comparison and online activation.
- [License Agreement](License-Agreement.md) - Full legal terms.
- [Command-Line Interface](Command-Line-Interface.md) - License activation flags (online).
