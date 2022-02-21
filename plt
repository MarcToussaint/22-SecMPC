set style data lines

#plot 'z.dat' 0:1 t 'time'

J = 14

plot \
'z.panda.dat' us 1:2 t 'real2', '' us 1:(column(J+2)) t 'ref2',\
'z.ref' us 1:3 t 'real2', '' us 1:(column(J+3)) t 'ref2',\
'z.ref2' us 1:3 t 'real2', '' us 1:(column(J+3)) t 'ref2'
exit

plot 'z.panda.dat' us 1:($2) t 'real1', '' us 1:(column(J+2)) t 'ref1',\
  '' us 1:($3) t 'real2', '' us 1:(column(J+3)) t 'ref2', \
  '' us 1:($4) t 'real3', '' us 1:(column(J+4)) t 'ref3', \
  '' us 1:($5) t 'real4', '' us 1:(column(J+5)) t 'ref4', \
  '' us 1:($6) t 'real5', '' us 1:(column(J+6)) t 'ref5', \
  '' us 1:($7) t 'real6', '' us 1:(column(J+7)) t 'ref6', \
  '' us 1:($8) t 'real7', '' us 1:(column(J+8)) t 'ref7', \

exit

plot 'z.panda0.dat' \
      us 0:($2-$9) t 'err1', \
   '' us 0:($3-$10) t 'err2', \
   '' us 0:($4-$11) t 'err3', \
   '' us 0:($5-$12) t 'err4', \
   '' us 0:($6-$13) t 'err5', \
   '' us 0:($7-$14) t 'err6', \
   '' us 0:($8-$15) t 'err7'

exit


#plot 'z.panda0.dat' us 1:($26) t 'u_cmd', \
#'' us 1:($33) t 'u_measured', \
#'' us 1:($40) t 'gravity', \
#'' us 1:($40+$26) t 'gravity+u_cmd', \



