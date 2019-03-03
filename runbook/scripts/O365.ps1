$credObject = Get-AutomationPSCredential -Name "M365b555148"
Connect-MsolService -Credential $credObject

$Session = New-PSSession –ConfigurationName Microsoft.Exchange -ConnectionUri https://outlook.office365.com/powershell-liveid -Credential $Credobject -Authentication Basic -AllowRedirection
Import-PSSession -Session $Session -DisableNameChecking:$true -AllowClobber:$true | Out-Null

$report = @()
$found=$false
$mailboxes = Get-Mailbox -RecipientTypeDetails UserMailbox -ResultSize Unlimited 
foreach ($mailbox in $mailboxes) {
    if (($mailbox.DeliverToMailboxAndForward) -or ($mailbox.forwardingsmtpaddress)) {
	    $report+=new-object psobject -property @{Mailbox = $mailbox.displayname;PrimarySMTP=$mailbox.primarysmtpaddress;Forwardingaddress=$mailbox.forwardingsmtpaddress;Forwarding=$mailbox.delivertomailboxandforward}      
        $found=$true
    }
}
If ($found){
    $reportout=$report | select mailbox,primarysmtp,forwardingaddress,forwarding | fl | out-string
} else {
    $reportout ="No forwards found"
}
Send-MailMessage -Credential $credObject -From "admin@m365b555418.onmicrosoft.com" -To "director@ciaops.com" -Subject "Office 365 Mailbox forward report" -Body $reportout -SmtpServer "outlook.office365.com" -UseSSL