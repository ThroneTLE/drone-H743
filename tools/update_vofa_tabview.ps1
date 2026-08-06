param(
    [string]$Path = 'E:\User\Doc\模板\tabs.tabview.json'
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path -LiteralPath $Path)) {
    throw "VOFA tabview not found: $Path"
}

$backup = "$Path.pre-compact.bak"
if (-not (Test-Path -LiteralPath $backup)) {
    Copy-Item -LiteralPath $Path -Destination $backup
}

# VOFA's importer is sensitive to its original JSON number formatting. Work
# from the byte-compatible backup and replace only the binding tokens.
$text = [System.IO.File]::ReadAllText($backup)
$channelMap = [ordered]@{
    22 = 7   # roll_rate_kd
    23 = 8   # pitch_rate_kd
    24 = 9   # yaw_angle_kp
    25 = 10  # yaw_rate_kd
    26 = 11  # pos_x_kp
    27 = 12  # pos_y_kp
    29 = 13  # vel_x_kd
    30 = 14  # vel_y_kd
    52 = 15  # pos_x_m
    53 = 16  # pos_y_m
    54 = 15  # pos_x_m
    55 = 18  # roll_angle_kp
    56 = 19  # pitch_angle_kp
    57 = 20  # pos_z_kp
    58 = 22  # vel_z_kd
    59 = 16  # pos_y_m
    60 = 17  # vel_loop_enable
}

$nameMap = [ordered]@{
    'vel_x_kd'           = 'pos_x_kp'
    'vel_y_kd'           = 'pos_y_kp'
    'accel_xy_limit_m_s2' = 'vel_x_kd'
    'accel_z_limit_m_s2'  = 'vel_y_kd'
    'vel_loop_x_kp'      = 'pos_x_m'
    'vel_loop_y_kp'      = 'pos_y_m'
    'vel_loop_out'       = 'pos_x_m_b'
    'vel_loop_x_ki'      = 'roll_angle_kp'
    'vel_loop_y_ki'      = 'pitch_angle_kp'
    'vel_loop_i'         = 'pos_z_kp'
    'vel_loop_x_kd'      = 'vel_z_kd'
    'vel_loop_y_kd'      = 'pos_y_m_b'
}

function Replace-UniqueToken([string]$Source, [string]$Old, [string]$New) {
    $count = ([regex]::Matches($Source, [regex]::Escape($Old))).Count
    if ($count -ne 1) {
        throw "Expected one '$Old' token, found $count"
    }
    return $Source.Replace($Old, $New)
}

foreach ($entry in $channelMap.GetEnumerator()) {
    $text = Replace-UniqueToken $text `
        ('"ch_menu":{"ctx":' + $entry.Key + '}') `
        ('"ch_menu":{"ctx":' + $entry.Value + '}')
}

foreach ($entry in $nameMap.GetEnumerator()) {
    $text = Replace-UniqueToken $text `
        ('"name_":"' + $entry.Key + '"') `
        ('"name_":"' + $entry.Value + '"')
}

$text = Replace-UniqueToken $text '"lines":[34,35]' '"lines":[5,6]'
$text = Replace-UniqueToken $text `
    '"indecies":[10,10,10,10,-1,-1]' `
    '"indecies":[4,4,4,4,-1,-1]'

$utf8NoBom = [System.Text.UTF8Encoding]::new($false)
[System.IO.File]::WriteAllText($Path, $text, $utf8NoBom)

# Parse only for validation; never serialize this object back to disk.
$json = $text | ConvertFrom-Json
$widgets = @($json.ctx.tabs | ForEach-Object { $_.widgets })
$sliders = @($widgets | Where-Object { $_.path -eq 'slider' })
$customNameSliders = @($sliders | Where-Object {
    $_.ctx.name_menu.ctx.attr.visible -and
    -not $_.ctx.name_menu.ctx.attr.name_link_ch -and
    -not $_.ctx.name_menu.ctx.attr.name_link_cmd -and
    -not [string]::IsNullOrWhiteSpace($_.ctx.name_menu.ctx.attr.name_)
})

if (($sliders.Count -ne 17) -or ($customNameSliders.Count -ne 17)) {
    throw "Slider custom-name validation failed"
}

$sliderNames = @($customNameSliders | ForEach-Object { $_.ctx.name_menu.ctx.attr.name_ })
foreach ($oldName in @('accel_xy_limit_m_s2', 'accel_z_limit_m_s2',
                       'vel_loop_x_kp', 'vel_loop_y_kp', 'vel_loop_out',
                       'vel_loop_x_ki', 'vel_loop_y_ki', 'vel_loop_i',
                       'vel_loop_x_kd', 'vel_loop_y_kd')) {
    if ($sliderNames -contains $oldName) {
        throw "Old slider name remains: $oldName"
    }
}
foreach ($newName in @('pos_x_kp', 'pos_y_kp', 'vel_x_kd', 'vel_y_kd',
                      'vel_loop_enable', 'roll_angle_kp', 'pitch_angle_kp',
                      'pos_z_kp', 'vel_z_kd')) {
    if ($sliderNames -notcontains $newName) {
        throw "Missing slider name: $newName"
    }
}

Write-Output "Updated $Path with byte-preserving replacements"
Write-Output "Backup $backup"
Write-Output "Sliders 17; wave channels 5,6; wave time channel 4"
