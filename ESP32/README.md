1.0 verzió elkészült


Jelenlegi program USB komonikációra készült. És véglegesnek tekinthető apró javítások várhatók még valamint később említve...<br>
Csak is az álltalam módosított programmal működik https://github.com/MrPoke21/AgOpenGPS/releases <br>
A rendszer nincs megkötve semmilyen téren bármiféle GPS modulal használható. Az RX lábon olvassa az NMEA adatokat a TX lábon pedig küldi az RTCM adatot.<br>
Alapvetően IBT2 motorvezérlőre van feliratozva a lábak kimenete, de a cytron vezérlő is használható, annyival van bővítve az official verzióhoz képest hogy az EN lábra kötött relé mosfettel teljesen le lehet vállasztani a motorvezérlőt.<br>
A panel tökéletesen alkalmas a hidraulikus vezértömbök kezelésére. Csak 3db PWM mosfetre van szükség PL
![kép](https://github.com/user-attachments/assets/22d2442e-f5ab-4dc8-b6c0-d2c29f04e67f)

További fejlesztések ami várható egy webes felület a paraméterek finomhangolásához.
Valamint egy CANBUS vezérlő a kormánymotor használatához, ehez persze hozzá kell jutnom egyhez, hogy normálisan le tudjam programozni, és ne olyan buta legyen mint a jelenlegi AGOpen kódokban.



