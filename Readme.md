<html>
<head><title>DAB Clock Radio Retrofit</title></head>
<body>

# DAB Clock Radio Retrofit
<p>
Inspired by a project on Hackday (<a href="http://hackaday.com/2014/09/18/sprite_tm-puts-linux-in-a-clock-radio/">Hackaday Link</a> and <a href="http://spritesmods.com/?art=clockradio&page=1">Original Source Link</a>) I replaced the internals
of my clock radio.
</p>
<p>
There were 2 problems with my aging clock radio - 1) no battery back-up after a power loss, and 2) poor radio tuning through the analog tuner.
I tried using my Samsung tablet, but it doesn't work very well as a clock - being an LCD, the backlight illuminates the whole room.
</p>

## Display 1

## Display 2

## Amplifier

## Keypad
<p>
I wanted to keep the enclosure and reuse as many of the buttons I could.  Most buttons are on one PCB with a rubber keys.
I took a photo of the board, which was single sided.  Below is a trace of the tracks...
</p>
<img src="images/keypad.png" style="widows:1112px; height:258px;" />
<p>
Lines labeled A1 (purple), A3 (red), and A5 (blue) are the the common lines in a strobed layout, however, they same lines are 
also sometimes used as inputs, eg When A1 is strobed and key 9 is pressed, A5 can be read.

## Encoder
<p>The am/fm tuning mechanism, is a disk type dial attached to a spindle, connected to the frequency display and variable capacitor via 
some string.  Through pure luck, I could replace the spindle with a mechanical encoder.

* DAB module

## Other Keys and switches
<p>The clock radio also has...</p>
* 2 volume sliders (analog inputs)
* 2, 3 position slide switches on the top
* 2, 2 posotion slide switches on the left and right

<p>I need to work out how I am going to use them.</p>

</body>
</html>