# What still has to be implemented

1) 	Extend OLJFSystem.lft to a interconnection between 2 OLJF systems where the interconnection is a CLJF system
1a)	In order to achieve this: make a new function: convertFilter2JF such that a filter is converted to a OLJF system which is used by lft
1b) Extend the append weighting filters for OL/CL SD and JF, not only for unweighted functions
1c) Add a addition and substraction (parralel connection to JF/SD)
2)	Implement options to choose the control strategy wished to be used: Diss: Quad, L2, H2, H2g, L1 or LQ(R/G) etc.
2a)	Add checks to make sure that some matrices are positive (semi) definite or negitive (semi) definite
2b)	Add line search for Hinf=L2-ind and add golden search for L1
2b)	For H2g used a search to find P(\tau)