# Set this to the port reported by `foundry server status`.
$port = 63572

# Request the list of models available from the local Foundry server.
Invoke-RestMethod -Uri "http://127.0.0.1:$port/v1/models"