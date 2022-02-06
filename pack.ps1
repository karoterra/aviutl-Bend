$archiveName = "aviutl-Bend"
$gitTag = (git tag --points-at)
if (![string]::IsNullOrEmpty($gitTag))
{
    $archiveName = "${archiveName}_${gitTag}"
}

New-Item publish -ItemType Directory -Force

7z a "publish\${archiveName}.zip" `
    .\README.md `
    .\CHANGELOG.md `
    .\LICENSE `
    ".\script\@曲げKR.anm" `
    ".\script\KaroterraBend.lua"
