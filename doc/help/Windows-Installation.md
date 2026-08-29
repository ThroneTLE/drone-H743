# Windows installation

## Overview

Serial Studio ships three ways on Windows, all 64-bit (x64) builds of the same application.
The MSI installer and the portable ZIP come from the
[releases page](https://github.com/Serial-Studio/Serial-Studio/releases/latest); the
Microsoft Store edition installs and updates through the Store.

| Channel         | File name / source                                  | Best for |
|-----------------|-----------------------------------------------------|----------|
| MSI installer   | `Serial-Studio-Pro-<version>-Windows.msi`           | Standard install with a Start Menu entry |
| Portable ZIP    | `Serial-Studio-Pro-<version>-Windows-Portable.zip`  | Running without installation or administrator rights |
| Microsoft Store | [Serial Studio Pro on the Microsoft Store](https://apps.microsoft.com/detail/9n5pzlkjcdp7) | Automatic updates managed by Windows |

All three require Windows 10 version 1809 or later. There is no 32-bit or ARM64 build.

## Code signing

The MSI installer and the portable ZIP are not code-signed yet, so Microsoft Defender
SmartScreen flags them as coming from an unknown publisher on first run. Click **More info**,
then **Run anyway** to proceed; the downloads themselves are served over HTTPS from GitHub.
The Microsoft Store package is signed by Microsoft as part of Store publishing, so Store
installs start without a SmartScreen prompt.

## MSI installer

1. Download `Serial-Studio-Pro-<version>-Windows.msi` from the
   [releases page](https://github.com/Serial-Studio/Serial-Studio/releases/latest).
2. Double-click the file and follow the setup wizard; the license text is displayed during
   installation.
3. Launch **Serial Studio Pro** from the Start Menu.

Setup places the application under `C:\Program Files\Serial Studio Pro` and adds a
**Serial Studio Pro** shortcut directly to the Start Menu. Installing a newer MSI upgrades
the existing installation in place; two versions never sit side by side. Uninstall from
**Settings → Apps → Installed apps**.

## Portable ZIP

Extract `Serial-Studio-Pro-<version>-Windows-Portable.zip` anywhere (a USB stick works) and
start the application with the `Serial Studio Pro.bat` launcher at the root of the archive,
or run the executable directly:

```text
Serial Studio Pro.bat                  launcher
License.rtf
Serial-Studio-Pro-v<version>\
  bin\Serial-Studio-Pro.exe            the application
  bin\ss-config.json
  plugins\, qml\, ...                  Qt runtime
```

Extraction writes nothing outside the folder and needs no administrator rights. Settings and
license activation are still stored per user in the Windows registry, so moving the folder
to another machine does not carry your configuration with it.

## Microsoft Store

Install [Serial Studio Pro from the Microsoft Store](https://apps.microsoft.com/detail/9n5pzlkjcdp7).
It is the same application packaged as MSIX: Windows installs it without an elevation
prompt and keeps it updated automatically through the Store.

## Updates

Each package embeds a stamp identifying its format, and the built-in update check uses it to
offer the matching package: an MSI installation is notified about new MSI releases, the
portable ZIP about new ZIP releases, and Store installations update through the Store.

## First launch

If the application fails to start on a fresh Windows installation, install the
[Microsoft Visual C++ Redistributable (64-bit)](https://aka.ms/vs/17/release/vc_redist.x64.exe)
and launch it again.

Devices that use a USB-to-serial bridge need the vendor driver before Windows shows a COM
port: [CH340](http://www.wch-ic.com/downloads/CH341SER_EXE.html),
[FTDI](https://ftdichip.com/drivers/vcp-drivers/), or
[CP210x](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers). Open Device
Manager and check under **Ports (COM & LPT)** to confirm the device enumerates. More
first-connection fixes are in [Getting Started](Getting-Started.md) and
[Troubleshooting](Troubleshooting.md).

## See also

- [Getting Started](Getting-Started.md): first connection walkthrough for all platforms.
- [Linux Installation](Linux-Installation.md): package formats and signature verification on
  Linux.
- [Troubleshooting](Troubleshooting.md): fixes for common connection and parsing problems.
- [Command Line Interface](Command-Line-Interface.md): flags for running Serial Studio from
  scripts and headless environments.
