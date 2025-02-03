Projekt Microchip Studio

Kromě Microchip Studia můžete použít třeba VScode.

Na nahrávání programu použijte např. https://github.com/mariusgreuel/avrdude/releases 

Nebo si můžete do VScode nastavit nahrávání přes avrdude:
    https://blog.zakkemble.net/avrdudess-a-gui-for-avrdude/


Nahrání programu test3.hex přes Xnano416 do ATtiny814:
avrdude.exe -p attiny814 -c xplainedmini_updi -U flash:w:test3.hex

Totéž pokud není nastavená cesta k AVRdude (pokud ho máte na uvedené cestě):
"C:\Program Files (x86)\avrdude\avrdude.exe" -p attiny814 -c xplainedmini_updi -U flash:w:test3.hex
