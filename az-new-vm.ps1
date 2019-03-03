<# CIAOPS
Script provided as is. Use at own risk. No guarantees or warranty provided.

Description
Script designed to create a standard Windows Server VM

Prerequisites = 1
1. Ensure azurerm module installed or updated

ensure that install-azurerm msonline has been run
ensure that update-azurerm msonline has been run to get latest module
Module = https://www.powershellgallery.com/packages/AzureRM/
Latest version 6.13.1 21 November 2018

Allow custom scripts to run just for this instance
set-executionpolicy -executionpolicy bypass -scope currentuser -force
#>

## Variables
$systemmessagecolor = "cyan"
$processmessagecolor = "green"
$errormessagecolor = "red"
$warnmessagecolor = "yellow"

$rgname = "demo1" ## new ARM resource group name
$location = "southeastasia" ## Azure region where new VM will reside
$vnetiprange ="10.2.0.0/16" ## VNET IP network
$vnetname="demo-net" ## new ARM VNET name
$vnetsubnetname="subnet1" ## new ARM VNET subnet in which VM will live
$vnetsubnetiprange="10.2.0.0/24" ## VNET IP subnet
$vnetdns="10.2.0.4" ## DNS for VNET
$storageacctname ="demo1store" ## for VHDs. Note must be globally unique
$nic_name="demo1_nic" ## for network card
$storage_type="Standard_LRS" ## storage account type

$vm_size="Standard_DS2_v2" ## VM size
$dc_name="server" ## new VM name
$os_sku="2008-R2-sp1" ## new VM OS
$osdiskname="sposdisk" ## new VM VHD name
$location="southeastasia"

Clear-Host

write-host -foregroundcolor $systemmessagecolor "Script started`n"

Get-AzureRMVMImagePublisher -Location $location | Select PublisherName

$pubName="MicrosoftWindowsServer"
Get-AzureRMVMImageOffer -Location $location -Publisher $pubName | Select Offer

$offerName="WindowsServer"
Get-AzureRMVMImageSku -Location $location -Publisher $pubName -Offer $offerName | Select Skus

### Create a new, empty Resource Group inside our Azure subscription.
New-AzureRmResourceGroup -Name $rgname -Location $location -Force

## Create a new storage account for disks
$storageAccount = New-AzureRmStorageAccount -name $storageacctname -resourcegroupname $rgname -type $storage_type -location $location

## Create Vnet

## Create a Subnet in Vnet
$subnetconfig=new-azurermvirtualnetworksubnetconfig -name $vnetname -addressprefix $vnetsubnetiprange

## Create a new Vnet in Resource Group
$vnet=New-AzureRmVirtualNetwork -ResourceGroupName $rgname -Name $vnetname -AddressPrefix $vnetiprange -Location $location -subnet $subnetconfig  

## Set Vnet DNS server
$vnet.DhcpOptions.DnsServers = $vnetdns

##Save changes to Vnet
Set-AzureRmVirtualNetwork -VirtualNetwork $vnet

# Create a public IP address and specify a DNS name
$pip = New-AzureRmPublicIpAddress -ResourceGroupName $rgname -Location $location -AllocationMethod Static -IdleTimeoutInMinutes 4 -Name "impublicdns$(Get-Random)"

## create VM

# Create an inbound network security group rule for port 3389
$nsgRuleRDP = New-AzureRmNetworkSecurityRuleConfig -Name myNetworkSecurityGroupRuleRDP  -Protocol Tcp -Direction Inbound -Priority 1000 -SourceAddressPrefix * -SourcePortRange * -DestinationAddressPrefix * -DestinationPortRange 3389 -Access Allow

# Create a network security group
$nsg = New-AzureRmNetworkSecurityGroup -ResourceGroupName $rgname -Location $location -Name myNetworkSecurityGroup -SecurityRules $nsgRuleRDP

# Create a virtual network card and associate with public IP address and NSG
$nic = New-AzureRmNetworkInterface -Name $nic_name -ResourceGroupName $rgname -Location $location -SubnetId $vnet.Subnets[0].Id -PublicIpAddressId $pip.Id -NetworkSecurityGroupId $nsg.Id -privateipaddress $vnetdns

# Define a credential object
$cred = Get-Credential

# Define disk URL
$dcosDiskUri = $storageAccount.PrimaryEndpoints.Blob.ToString() + "vhds/" + $dc_Name + $osDiskName + ".vhd"

# Create a virtual machine configuration
$vmConfig = New-AzureRmVMConfig -VMName $dc_name -VMSize $vm_size 
$vmconfig = set-azurermvmosdisk -vm $vmconfig -name $osdiskname -createoption "fromimage" -vhduri $dcosdiskuri
$vmConfig = Set-AzureRmVMOperatingSystem -VM $vmconfig -Windows -ComputerName $dc_name -Credential $cred
$vmConfig = Set-AzureRmVMSourceImage -VM $vmconfig -PublisherName MicrosoftWindowsServer -Offer WindowsServer -Skus $os_sku -Version latest
$vmConfig = Add-AzureRmVMNetworkInterface -VM $vmconfig -Id $nic.Id

New-AzureRmVM -ResourceGroupName $rgname -Location $location -VM $vmConfig

write-host -foregroundcolor $systemmessagecolor "Script finished"