# PA3 Report

B08901051 電機四 王濰紳

## 1)  tdfsim.cpp and experimental results

| circuit number | # gates | # total TDFs | # detected faults | # undetected faults | transition delay fault coverage |
| :----: | :----: | :----: | :----: | :----: | :----: |
|  C17   |     6  |    34  |    23  |    11  |  67.64%  |
|  C432  |   245  |  1110  |     3  |  1107  |  00.27%  |
|  C499  |   554  |  2390  |  1552  |   838  |  64.93%  |
|  C880  |   545  |  2104  |   792  |  1312  |  37.64%  |
|  C1355 |   554  |  2726  |   593  |  2133  |  21.75%  |
|  C2670 |  1785  |  6520  |  4668  |  1852  |  71.59%  |
|  C3540 |  2082  |  7910  |  1142  |  6768  |  14.43%  |
|  C6288 |  4800  | 17376  | 16532  |   844  |  95.14%  |
|  C7552 |  5679  | 19456  | 17421  |  2035  |  89.54%  |


The following code include four `ATPG` functions, `transition_delay_fault_simultation`, `tdffault_sim_a_vector`, `tdffault_sim_a_vector2` and `generate_tdfault_list`.
The `generate_tdfault_list` will be called before the fault simulation phase and will traverse through all the gates with no fault collapsing. The `transition_delay_fault_simulation` will be called during the fault simnulation phase. It will iterate through all the given test patterns and perform parallel fault simulation. First, it calls `tdf_sim_a_vector` and perform good simluation with vec1 to see if the fault is activated and mark the fault activated by changing the `fault->activate` to 1(initially 0). Second, it calls `tdf_sim_a_vector2` and perform parallel fault simulation with vec2 to see if the activated faults are able to be propagated to the output (detected), and it will drop detected faults.

```c++
void ATPG::transition_delay_fault_simulation(int &total_detect_num)
{
	int current_activated_num = 0;
	int current_detect_num = 0;
	for (int i = this->vectors.size() - 1; i >= 0; i--)
	{
		tdfault_sim_a_vector(vectors[i], current_activated_num);
		tdfault_sim_a_vector2(vectors[i], current_detect_num);
		total_detect_num += current_detect_num;
		fprintf(stdout, "vector[%d] detects %d faults (%d)\n", i, current_detect_num, total_detect_num);
	}
}

void ATPG::tdfault_sim_a_vector(const string &vec, int &num_of_current_activate)
{
	fptr f;
	int nckt = this->sort_wlist.size();

	num_of_current_activate = 0;

	/* for every input, set its value to the current vector value */
	for (int i = 0; i < this->cktin.size(); i++)
	{
		this->cktin[i]->value = ctoi(vec[i]);
	}

	/* initialize the circuit - mark all inputs as changed and all other
	 * nodes as unknown (2) */
	for (int i = 0; i < nckt; i++)
	{
		if (i < this->cktin.size())
		{
			this->sort_wlist[i]->set_changed();
		}
		else
		{
			this->sort_wlist[i]->value = U;
		}
	}

	sim(); /* do a fault-free simulation, see sim.cpp */
	if (debug)
	{
		display_io();
	}

	/* walk through every undetected fault
	 * the undetected fault list is linked by pnext_undetect */
	for (fptr f : this->flist_undetect)
	{

		if (f->fault_type == this->sort_wlist[f->to_swlist]->value)
		{
			f->activate = 1;
			num_of_current_activate++;
		}
		else
		{
			f->activate = 0;
		}
	} // end loop. for f = flist
} /* end of fault_sim_a_vector */

void ATPG::tdfault_sim_a_vector2(const string &vec, int &num_of_current_detect)
{
	wptr w, faulty_wire;
	/* array of 16 fptrs, which points to the 16 faults in a simulation packet  */
	fptr simulated_fault_list[num_of_tdfault];
	fptr f;
	int fault_type;
	int i, start_wire_index, nckt;
	int num_of_fault;

	num_of_fault = 0; // counts the number of faults in a packet

	/* num_of_current_detect is used to keep track of the number of undetected faults
	 * detected by this vector.  Initialize it to zero */
	num_of_current_detect = 0;

	/* Keep track of the minimum wire index of 16 faults in a packet.
	 * the start_wire_index is used to keep track of the
	 * first gate that needs to be evaluated.
	 * This reduces unnecessary check of scheduled events.*/
	start_wire_index = 10000;

	/* for every input, set its value to the current vector value */
	for (i = 0; i < cktin.size(); i++)
	{
		if (i == 0)
		{
			cktin[i]->value = ctoi(vec[cktin.size()]);
		}
		else
		{
			cktin[i]->value = ctoi(vec[i - 1]);
		}
	}

	/* initialize the circuit - mark all inputs as changed and all other
	 * nodes as unknown (2) */
	nckt = sort_wlist.size();
	for (i = 0; i < nckt; i++)
	{
		if (i < cktin.size())
		{
			sort_wlist[i]->set_changed();
		}
		else
		{
			sort_wlist[i]->value = U;
		}
	}

	sim(); /* do a fault-free simulation, see sim.cpp */
	if (debug)
	{
		display_io();
	}

	/* expand the fault-free value into 32 bits (00 = logic zero, 11 = logic one, 01 = unknown)
	 * and store it in wire_value1 (good value) and wire_value2 (faulty value)*/
	for (i = 0; i < nckt; i++)
	{
		switch (sort_wlist[i]->value)
		{
			case 1:
				sort_wlist[i]->wire_value1 = ALL_ONE; // 11 represents logic one
				sort_wlist[i]->wire_value2 = ALL_ONE;
				break;
			case 2:
				sort_wlist[i]->wire_value1 = 0x55555555; // 01 represents unknown
				sort_wlist[i]->wire_value2 = 0x55555555;
				break;
			case 0:
				sort_wlist[i]->wire_value1 = ALL_ZERO; // 00 represents logic zero
				sort_wlist[i]->wire_value2 = ALL_ZERO;
				break;
		}
	} // for in

	/* walk through every undetected fault
	 * the undetected fault list is linked by pnext_undetect */
	for (auto pos = flist_undetect.cbegin(); pos != flist_undetect.cend(); ++pos)
	{
		int fault_detected[num_of_tdfault] = {0}; // for n-det
		f = *pos;
		if (f->detect == REDUNDANT)
		{
			continue;
		} /* ignore redundant faults */

		/* consider only active (aka. excited) fault
		 * (sa1 with correct output of 0 or sa0 with correct output of 1) */
		if (f->fault_type != sort_wlist[f->to_swlist]->value && f->activate == 1)
		{

			/* if f is a primary output or is directly connected to an primary output
			 * the fault is detected */
			if ((f->node->type == OUTPUT) ||
					(f->io == GO && sort_wlist[f->to_swlist]->is_output()))
			{
				f->detected_time++;
				if (f->detected_time == detected_num)
				{
					f->detect = TRUE;
				}
			}
			else
			{

				/* if f is an gate output fault */
				if (f->io == GO)
				{

					/* if this wire is not yet marked as faulty, mark the wire as faulty
					 * and insert the corresponding wire to the list of faulty wires. */
					if (!(sort_wlist[f->to_swlist]->is_faulty()))
					{
						sort_wlist[f->to_swlist]->set_faulty();
						wlist_faulty.push_front(sort_wlist[f->to_swlist]);
					}

					/* add the fault to the simulated fault list and inject the fault */
					simulated_fault_list[num_of_fault] = f;
					inject_fault_value(sort_wlist[f->to_swlist], num_of_fault, f->fault_type);

					/* mark the wire as having a fault injected
					 * and schedule the outputs of this gate */
					sort_wlist[f->to_swlist]->set_fault_injected();
					for (auto pos_n : sort_wlist[f->to_swlist]->onode)
					{
						pos_n->owire.front()->set_scheduled();
					}

					/* increment the number of simulated faults in this packet */
					num_of_fault++;
					/* start_wire_index keeps track of the smallest level of fault in this packet.
					 * this saves simulation time.  */
					start_wire_index = min(start_wire_index, f->to_swlist);
				} // if gate output fault

				/* the fault is a gate input fault */
				else
				{

					/* if the fault is propagated, set faulty_wire equal to the faulty wire.
					 * faulty_wire is the gate output of f.  */
					faulty_wire = get_faulty_wire(f, fault_type);
					if (faulty_wire != nullptr)
					{

						/* if the faulty_wire is a primary output, it is detected */
						if (faulty_wire->is_output())
						{
							f->detected_time++;
							if (f->detected_time == detected_num)
							{
								f->detect = TRUE;
							}
						}
						else
						{
							/* if faulty_wire is not already marked as faulty, mark it as faulty
							 * and add the wire to the list of faulty wires. */
							if (!(faulty_wire->is_faulty()))
							{
								faulty_wire->set_faulty();
								wlist_faulty.push_front(faulty_wire);
							}

							/* add the fault to the simulated list and inject it */
							simulated_fault_list[num_of_fault] = f;
							inject_fault_value(faulty_wire, num_of_fault, fault_type);

							/* mark the faulty_wire as having a fault injected
							 *  and schedule the outputs of this gate */
							faulty_wire->set_fault_injected();
							for (auto pos_n : faulty_wire->onode)
							{
								pos_n->owire.front()->set_scheduled();
							}

							num_of_fault++;
							start_wire_index = min(start_wire_index, f->to_swlist);
						}
					}
				}
			} // if  gate input fault
		}		// if fault is active

		/*
		 * fault simulation of a packet
		 */

		/* if this packet is full (16 faults)
		 * or there is no more undetected faults remaining (pos points to the final element of flist_undetect),
		 * do the fault simulation */
		if ((num_of_fault == num_of_tdfault) || ((next(pos, 1) == flist_undetect.cend()) && num_of_fault > 0))
		{

			/* starting with start_wire_index, evaulate all scheduled wires
			 * start_wire_index helps to save time. */
			for (i = start_wire_index; i < nckt; i++)
			{
				if (sort_wlist[i]->is_scheduled())
				{
					sort_wlist[i]->remove_scheduled();
					fault_sim_evaluate(sort_wlist[i]);
				}
			} /* event evaluations end here */

			/* pop out all faulty wires from the wlist_faulty
			 * if PO's value is different from good PO's value, and it is not unknown
			 * then the fault is detected.
			 *
			 * IMPORTANT! remember to reset the wires' faulty values back to fault-free values.
			 */
			while (!wlist_faulty.empty())
			{
				w = wlist_faulty.front();
				wlist_faulty.pop_front();
				w->remove_faulty();
				w->remove_fault_injected();
				w->set_fault_free();
				/* TODO */

				/*
				 * After simulation is done,if wire is_output(), we should compare good value(wire_value1) and faulty value(wire_value2).
				 * If these two values are different and they are not unknown, then the fault is detected.  We should update the simulated_fault_list.  Set detect to true if they are different.
				 * Since we use two-bit logic to simulate circuit, you can use Mask[] to perform bit-wise operation to get value of a specific bit.
				 * After that, don't forget to reset faulty values (wire_value2) to their fault-free values (wire_value1).
				 */
				if (w->is_output())
				{ // if primary output
					for (i = 0; i < num_of_fault; i++)
					{ // check every undetected fault
						if (!(simulated_fault_list[i]->detect))
						{
							if ((w->wire_value2 & Mask[i]) ^ // if value1 != value2
									(w->wire_value1 & Mask[i]))
							{
								if (((w->wire_value2 & Mask[i]) ^ Unknown[i]) && // and not unknowns
										((w->wire_value1 & Mask[i]) ^ Unknown[i]))
								{
									fault_detected[i] = 1; // then the fault is detected
								}
							}
						}
					}
				}
				w->wire_value2 = w->wire_value1; // reset to fault-free values
																				 /* end TODO*/
			}																	 // pop out all faulty wires
			// for n-det
			for (i = 0; i < num_of_fault; i++)
			{
				if (fault_detected[i] == 1)
				{
					simulated_fault_list[i]->detected_time++;
					if (simulated_fault_list[i]->detected_time == detected_num)
					{
						simulated_fault_list[i]->detect = TRUE;
					}
				}
			}
			num_of_fault = 0;					// reset the counter of faults in a packet
			start_wire_index = 10000; // reset this index to a very large value.
		}														// end fault sim of a packet
	}															// end loop. for f = flist

	/* fault dropping  */
	flist_undetect.remove_if(
			[&](const fptr fptr_ele)
			{
				if (fptr_ele->detect == TRUE)
				{
					num_of_current_detect += fptr_ele->eqv_fault_num;
					return true;
				}
				else
				{
					return false;
				}
			});
} /* end of fault_sim_a_vector */

void ATPG::generate_tdfault_list()
{
	int fault_num;
	wptr w;
	nptr n;
	fptr_s f;
	fptr_s f_at_in;
	for (wptr w : sort_wlist)
	{
		n = w->inode.front();

		/* for each gate, create a gate output stuck-at zero (SA0) fault */
		f = move(fptr_s(new (nothrow) FAULT));
		if (f == nullptr)
			error("No more room!");
		f->node = n;

		f->io = GO;
		f->fault_type = STR;
		f->to_swlist = w->wlist_index;
		f->eqv_fault_num = 1;
		if (n->type == OUTPUT)
		{
			continue;
		}
		if (n->type == NOT || n->type == BUF)
		{
			for (wptr iw : n->iwire)
			{
				if (iw->onode.size() > 1)
				{
					f->eqv_fault_num++;
				}
			}
		}

		num_of_gate_fault += f->eqv_fault_num; // accumulate total uncollapsed faults
		flist_undetect.push_front(f.get());		 // initial undetected fault list contains all faults
		flist.push_front(move(f));						 // push into the fault list

		/* for each gate, create a gate output stuck-at one (SA1) fault */
		f = move(fptr_s(new (nothrow) FAULT));

		if (f == nullptr)
			error("No more room!");
		f->node = n;
		f->io = GO;
		f->fault_type = STF;
		f->to_swlist = w->wlist_index;
		f->eqv_fault_num = 1;
		if (n->type == NOT || n->type == BUF)
		{
			for (wptr iw : n->iwire)
			{
				if (iw->onode.size() > 1)
				{
					f->eqv_fault_num++;
				}
			}
		}
		num_of_gate_fault += f->eqv_fault_num; // accumulate total uncollapsed faults
		flist_undetect.push_front(f.get());		 // initial undetected fault list contains all faults
		flist.push_front(move(f));						 // push into the fault list

		/*if w has multiple fanout branches */
		if (w->onode.size() > 1)
		{
			for (nptr nptr_ele : w->onode)
			{
				/* create SA0 for OR NOR EQV XOR gate inputs  */
				if (nptr_ele->type == BUF || nptr_ele->type == NOT || nptr_ele->type == INPUT)
				{
					continue;
				}
				f = move(fptr_s(new (nothrow) FAULT));
				if (f == nullptr)
					error("No more room!");
				f->node = nptr_ele;
				f->io = GI;
				f->fault_type = STR;
				f->to_swlist = w->wlist_index;
				f->eqv_fault_num = 1;
				/* f->index is the index number of gate input,
					 which GI fault is associated with*/
				for (int k = 0; k < nptr_ele->iwire.size(); k++)
				{
					if (nptr_ele->iwire[k] == w)
						f->index = k;
				}
				num_of_gate_fault++;
				flist_undetect.push_front(f.get());
				flist.push_front(move(f));

				f = move(fptr_s(new (nothrow) FAULT));
				if (f == nullptr)
					error("No more room!");
				f->node = nptr_ele;
				f->io = GI;
				f->fault_type = STF;
				f->to_swlist = w->wlist_index;
				f->eqv_fault_num = 1;
				for (int k = 0; k < nptr_ele->iwire.size(); k++)
				{
					if (nptr_ele->iwire[k] == w)
						f->index = k;
				}
				num_of_gate_fault++;
				flist_undetect.push_front(f.get());
				flist.push_front(move(f));
			}
		}
	}

	this->flist.reverse();
	this->flist_undetect.reverse();
	/*walk through all faults, assign fault_no one by one  */
	fault_num = 0;
	num_of_tdf_fault = 0;
	for (fptr f : this->flist_undetect)
	{
		f->fault_no = fault_num;
		fault_num++;
		this->num_of_tdf_fault += f->eqv_fault_num;
	}
}
```

## 2) generating the transition delay fault list and the reason TDFs cannot be collapsed

The method of generating the transition delay fault list is similar to that of generating the stuck-at fault list, I made sure there were no fault injected in the input of a Buffer and Inverter gates and handled some edge cases. I did NOT collapse the TDFs because they cannot be collpased.

The reason TDFs cannot be collapsed is because, for any gates with two or more inputs there are no equivalent faults. Equivalent faults need to have same behavior under every possible pattern. For example, for an AND gate with inputA and inputB, A's STR and B's STR faults are not equivalent fault because 10 => 11(A1B1 => A2B2) can activate B's STR fault but cannot activate A's STR fault. On the other hand, in a similar scenario of stuck-at faults, assume we have A's and B's stuck-at 0 faults, they are equivalent fault because all possible pattern of AB(00, 01, 10, 11) have the same behavior with both faults, hence they are equivalent.
To conclude, if STR faults were to be equivalent and be collapsed both faults of a two-input gate need to have the same behavior under 16 different patterns with v1 and v2, which is impossible.