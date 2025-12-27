What were the major issues with the Philips project...

> Company A -> Subcontracted the electrical and the mechanical designs out to 2 seperate divisions... resulting in 
A lot of design confusion

> there was a 10 Pin connector attached to the ATI clearly designed for intechangable IO yet there were over 9 additional IO blocks added
1 for each individual component additionally... there were rediculous robot choices made as well

The main issue, positional drift over a "prolonged" period of time... the first couple of welds looked "alright but the main issue was that there was nothing that was really repeatable over time."

The further into the parts the system got the more drift that would occur, but the drift wasn't consistent at all the main issue being the there were so many different components that just didnt make sense we didn't know where to start

> We had no loading instructions, no cell instructions, code with no comments., there was no real logic or reson behind specific design choices, we couldn't get in touch with the OEM because quiite franlkly they had no interest in talking to anybody... especially after burning bridges with the Customer C.

> So imagine for a moment, you have a system with "no documentation" no real resources other than a half assed build that has some of the most redicioulous inconsistent problems...

> First thing we do... dumb the problem down... focus on the robots... specifically, how they talk to the positioner... this welding cell consists of 2 Kuka Robots, 1 Material Handling robot 1 Welding Robot... the opperator is meant to bring a pallet of fresh components, shuttle in a brand new "ROllup" metal (cylinder) for the MRI machine and weld these componets on to the exterior of the rollup by rotating an Axis, E1 to an expected position, based on how the actual rollup was loaded, by first... using a laser to check the edge of the "window" theres basically this square cuttout, and the goal was to determine where our start position was based off the inset of one of these windows.... from there the user is meant to essentially, bring the welding robot up to the outside of the big metal tube... wait for it to rotate to a given position... activate the laser... look for a change in distance... then rotate backwards from that change in distance to its... new zero position... the reason for this is because the Fixture was extremely poorly designed with the metal tube fitting several different metal cuttout variations... the 0 position would change from component to component... additionally... once the