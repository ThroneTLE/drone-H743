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
    52 = 11  # vel_loop_x_kp
    53 = 12  # vel_loop_y_kp
    55 = 13  # vel_loop_x_ki
    56 = 14  # vel_loop_y_ki
    58 = 15  # vel_loop_x_kd
    59 = 16  # vel_loop_y_kd
    60 = 17  # vel_loop_enable
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

if (($sliders.Count -ne 10) -or ($customNameSliders.Count -ne 10)) {
    throw "Slider custom-name validation failed"
}

Write-Output "Updated $Path with byte-preserving replacements"
Write-Output "Backup $backup"
Write-Output "Sliders 10; wave channels 5,6; wave time channel 4"
