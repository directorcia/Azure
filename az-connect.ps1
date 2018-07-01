Import-module azurerm
## Import-Module "C:\Program Files (x86)\Microsoft SDKs\Azure\PowerShell\ServiceManagement\Azure\Azure.psd1"
Login-azurermaccount

## Select ARM account
$subscription=get-azurermsubscription
$subscriptionname = $subscription.name[0]

Select-azurermsubscription -subscriptionname $subscriptionname