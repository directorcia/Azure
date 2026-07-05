
$ollamaIp = "192.168.1.230"
$uri = "http://${ollamaIp}:11434/api/generate"
$timeoutSec = 30
$promptText = "Reply with exactly: Hello from Ollama"

$body = @{
    model  = "llama3.1:8b"
    prompt = $promptText
    stream = $false
} | ConvertTo-Json

$stopwatch = [System.Diagnostics.Stopwatch]::StartNew()

try {
    $result = Invoke-RestMethod `
        -Uri $uri `
        -Method Post `
        -ContentType "application/json" `
        -Body $body `
        -TimeoutSec $timeoutSec `
        -ErrorAction Stop

    $stopwatch.Stop()
    $elapsedMs = [Math]::Round($stopwatch.Elapsed.TotalMilliseconds, 2)

    if ($null -eq $result) {
        Write-Error "Ollama returned no data."
        exit 1
    }

    if (($result.PSObject.Properties.Name -contains "response") -and -not [string]::IsNullOrWhiteSpace($result.response)) {
        Write-Output "Prompt: $promptText"
        Write-Output "Model reply: $($result.response)"
        Write-Output "Elapsed: ${elapsedMs} ms"
    }
    else {
        Write-Warning "Response did not include a non-empty 'response' field. Printing raw payload."
        Write-Output "Prompt: $promptText"
        Write-Output "Elapsed: ${elapsedMs} ms"
        $result | ConvertTo-Json -Depth 10
    }
}
catch {
    if ($stopwatch.IsRunning) {
        $stopwatch.Stop()
    }
    $elapsedMs = [Math]::Round($stopwatch.Elapsed.TotalMilliseconds, 2)
    Write-Error "Request to $uri failed (timeout: ${timeoutSec}s, elapsed: ${elapsedMs} ms). $($_.Exception.Message)"
    exit 1
}
