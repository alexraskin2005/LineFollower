# Proof of Concepts

De proof of concept bestaat uit 1 programma wegens ik verder heb gewerkt op de zelfde fille en vergeten ben apparte filles te maken.
<br />  
<br />
Deze proof of concept heeft de volgende functionaliteit
* Verbinden via wifi, indien de credentials uit het eeprom niet werken wordt er een configure portal opgezet om deze in te kunnen geven (esp32 in router mode)
* Eenmaal verbonden met wifi kan met linefollwer.local/set?m1= of /set?m2= de pwm output voor de motoren ingesteld worden
* Met linefollwer.local/start of /stop kunnen de motoren gestart en gestopt worden
* Met linefollwer.local/debug kunnen parameters uitgelezen worden, hiermee kunnen de sensor waarden bekeken worden
<br />
Hiermeer worden de volgende zaken aangetoond
* De draadloze communicatie werkt
* De motoren kunnen afzonderlijk aangestuurd worden
* De sensoren kunnen uitgelezen worden (hier mee ook te zien dat verschil tussen zwart en wit duidelijk is)
  
De elektronische schakelingen van de proof of concepts komt overeen met het elektrisch schema van plan B
