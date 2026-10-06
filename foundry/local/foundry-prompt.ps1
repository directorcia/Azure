# Set these to the model name and port reported by your Foundry server.
$model = "Phi-4-mini-instruct-cuda-gpu"
$port = 63572

# Build a chat request for the selected Foundry model and user prompt.
$body = @{
    model = $model
    messages = @(
        @{
            role = "user"
            content = "Hello"
        }
    )
} | ConvertTo-Json -Depth 10

# Send the request to Foundry's chat completions endpoint and display the response.
Invoke-RestMethod `
  -Uri "http://127.0.0.1:$port/v1/chat/completions" `
  -Method Post `
  -ContentType "application/json" `
  -Body $body