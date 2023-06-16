/**********************************************************************/
/*           This is the logic simulator for atpg                     */
/*                                                                    */
/*           Author: Bing-Chen (Benson) Wu                            */
/*           last update : 02/20/2023                                 */
/**********************************************************************/

#include "atpg.h"
#include "logic_tbl.h"

/*
 *   sim
 *
 *   RETURNS
 *       no value is returned
 *
 *   SIDE-EFFECTS
 *       changes the logic values inside some wires' data structures
 *
 *   perform the logic simulation
 *   event-driven technique coupled with levelling mechanism is employed
 *   single pattern, no fault injected
 */

void ATPG::sim()
{
	int ncktin, nckt;

	ncktin = cktin.size();
	nckt = sort_wlist.size();
	/* TODO */
	/* for every input */
	// Schedule every gate connected to a changed input.
	for (int inWireIndex = 0; inWireIndex < ncktin; ++inWireIndex)
	{
		if (cktin[inWireIndex]->is_changed())
		{
			cktin[inWireIndex]->remove_changed();
			cktin[inWireIndex]->set_scheduled();
		}
	}
	// evaluate() every scheduled gate & propagate any changes.
	// walk through all wires in increasing order.
	// Because the wires are sorted according to their levels,
	// it is correct to evaluate the wires in increasing order.
	/*
	 * For event-driven simulation, we set_change() after we inject new values (all ncktin wires)
	 * So, if a wire value is_changed(), we should clear this flag by remove_changed()
	 *  and set_schedule() to wait for simulation.
	 * We can then start to simulate circuit by sort_wlist, which it is sorted by level.
	 * Use evaluate() on connected node and schedule next wire if value changed.
	 */
	for (int wireIndex = 0; wireIndex < nckt; ++wireIndex)
	{
		if (sort_wlist[wireIndex]->is_changed())
		{
			sort_wlist[wireIndex]->remove_changed();
			sort_wlist[wireIndex]->set_scheduled();
		}
		if (sort_wlist[wireIndex]->is_scheduled())
		{
			sort_wlist[wireIndex]->remove_scheduled();
			for (int gateIndex = 0; gateIndex < sort_wlist[wireIndex]->onode.size(); ++gateIndex)
			{
				if (sort_wlist[wireIndex]->onode[gateIndex]->type != OUTPUT)
				{
					evaluate(sort_wlist[wireIndex]->onode[gateIndex]);
				}
			}
		}
	}
	// for(int i = 0;i<nckt;i++){
	// 	std::cerr << sort_wlist[i]->value;
	// }

	/*end of TODO*/
} /* end of sim */

void ATPG::evaluate(nptr n)
{
	int old_value, new_value;
	int i, nin;

	old_value = n->owire.front()->value;

	/* decompose a multiple-input gate into multiple levels of two-input gates
	 * then look up the truth table of each two-input gate
	 */
	nin = n->iwire.size();
	switch (n->type)
	{
		case AND:
		case BUF:
		case NAND:
			new_value = 1;
			for (i = 0; i < nin; i++)
			{
				new_value = ANDTABLE[n->iwire[i]->value][new_value];
			}
			if (n->type == NAND)
			{
				new_value = INV[new_value];
			}
			break;
		case OR:
		case NOR:
			new_value = 0;
			for (i = 0; i < nin; i++)
			{
				new_value = ORTABLE[n->iwire[i]->value][new_value];
			}
			if (n->type == NOR)
			{
				new_value = INV[new_value];
			}
			break;
		case NOT:
			new_value = INV[n->iwire.front()->value];
			break;
		case XOR:
			new_value = XORTABLE[n->iwire[0]->value][n->iwire[1]->value];
			break;
		case EQV:
			new_value = INV[(XORTABLE[n->iwire[0]->value][n->iwire[1]->value])];
			break;
	}
	if (old_value != new_value)
	{
		n->owire.front()->set_changed();
		n->owire.front()->value = new_value;
	}
} /* end of evaluate */

int ATPG::ctoi(const char &c)
{
	if (c == '2')
		return (2);
	if (c == '1')
		return (1);
	if (c == '0')
		return (0);
}
