Cutout
------

Cutout was created using an NPTH pad, with hole and pad dimensions being identical.  Then, the following procedure
was used to copy this to the Edge.Cuts layer (while also retaining it in "All copper layers", or wherever the hole appears
by default).

https://stackoverflow.com/questions/43949808/create-a-footprint-for-mounting-through-the-board-cut-in-kicad

  (pad "" np_thru_hole oval (at 0 0) (size 12.5 1.2) (drill oval 12.5 1.2) (layers *.Cu Eco1.User))
  (pad "" np_thru_hole oval (at 0 0) (size 12.5 1.2) (drill oval 12.5 1.2) (layers *.Cu Edge.Cuts))


Mounting Holes
--------------

Two mounting holes were created as NPTH pads with diameter 0.9 mm.