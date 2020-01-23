<# CIAOPS
Script provided as is. Use at own risk. No guarantees or warranty provided.

Description - Change the tier l;evel of items inside an Azure storage account. The storage account will need to support those levels.
Source - https://github.com/directorcia/Azure/blob/master/az-blob-tierset.ps1

Source - https://github.com/directorcia/Office365/blob/master/graph-connect.ps1

Prerequisites = 2
1. Already connected to Azure - https://github.com/directorcia/Azure/blob/master/az-connect.ps1
2. Change the variable to match Storage Account name, Resource Group and desired tier

More scripts available by joining http://www.ciaopspatron.com

#>

## Variables
$systemmessagecolor = "cyan"
$processmessagecolor = "green"
$storageaccountname = "<your storage account name here>"
$storageresourcegroup = "<your storage account resource group name here>"
$storagetier = "<your desired storage tier level here>"     # Hot, Cool or Archive

clear-host

write-host -foregroundcolor $systemmessagecolor "Script started`n"

write-host -foregroundcolor $processmessagecolor "Get Storage Account"
$storageaccount = Get-AzStorageAccount -name $storageaccountname -ResourceGroupName $storageresourcegroup

write-host -foregroundcolor $processmessagecolor "Get Storage Account key"
$key = (get-azstorageaccountkey -ResourceGroupName $storageaccount.ResourceGroupName -Name $storageaccount.StorageAccountName).value[0]

write-host -foregroundcolor $processmessagecolor "Get Storage context"
$context = New-AzstorageContext -StorageAccountName $storageaccount.StorageAccountName -StorageAccountKey $key

write-host -foregroundcolor $processmessagecolor "Get Storage containers"
$storagecontainers = get-azstoragecontainer -Context $context

write-host -foregroundcolor $processmessagecolor "Get Items"
$Blobs = @()
foreach ($StorageContainer in $StorageContainers) {
    $Blobs += Get-AzStorageBlob -Context $Context -Container $StorageContainer.Name
}

write-host -foregroundcolor $processmessagecolor "Change tier of all items`n"
Foreach ($Blob in $Blobs) {
    #Set tier of all blobs to desired tier
    $blob.icloudblob.SetStandardBlobTier($StorageTier)
    write-host  "Changed -", $blob.name
}

write-host -foregroundcolor $systemmessagecolor "Script finished`n"
