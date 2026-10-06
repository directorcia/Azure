# Generate persistent key
# This script generates a persistent secret key using 32 random bytes and converts it to a Base64 string.

$bytes = New-Object byte[] 32
[System.Security.Cryptography.RandomNumberGenerator]::Fill($bytes)
$secret = [Convert]::
$secret

# Generate persistent key (32 random bytes)

$bytes = New-Object byte[] 32

[System.Security.Cryptography.RandomNumberGenerator]::Fill($bytes)
# Convert to Base64
$secret = [Convert]::ToBase64String($bytes)

Write-Host "Secret Key:"
Write-Host $secret