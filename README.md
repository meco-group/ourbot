## Scanmatch + Kalman + Motionplanning (Michiel)

Opdracht kan opgedeeld worden in 2 luiken:

### (globale) pose schatting op basis van scanmatching + kalmanschatter
  * Op basis van bestaande scanmatch en kalman-schatter
  * Kalman-schatter brengt information samen over:
    - Model + inputsignalen
    - Meting encoders
    - Meting accelerometers
    - Scanmatch resultaat (asynchroon)
    om zo toestand, waaronder globale pose, te schatten.
  * Scanmatcher schat pose tussen 2 gemeten puntenwolken, op basis van lidar data
  * Huidige implementatie van de scanmatcher bepaalt relatieve pose tussen 2 posities waarop 2 eenvolgende lidarmetingen werden gedaan
  * Voorstel om deze aan te passen zodat telkens lidarmetingen worden vergeleken met fictieve lidarmetingen vanuit dezelfde positie, berekend via een gekende globale kaart. Dit vereist de volgende stappen:
    - Vanuit omgeving, gemodelleerd bv. in `omg-tools`, een (xml) bestand schrijven met relevante info over obstakels en omgevingsgrenzen (vb. hoekpunten van polygone obstakels, middelpunt+straal van cirkels, ...)
    - Dit bestand moet vanuit de Scanmatch component uitgelezen worden en de data bewaard worden.
    - Functionaliteit schrijven zodat fictieve lidarmetingen wordt gegenereerd op basis van de kaart en huidige pose.
    - Scanmatch component verder aanpassen zodat de echte lidarmetingen vergeleken worden met fictieve lidarmetingen.
    - Kalman filter aanpassen: zie @@@Michiel comments in de code.
  * Dit alles kan getest worden door met de gamepad rond te rijden in een opgebouwde omgeving.

### link met motionplanning
  * Eenmaal de schatting in orde is kan deze gekoppeld worden met de motionplanning component.
