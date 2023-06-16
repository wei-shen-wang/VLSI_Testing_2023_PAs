# PA1 Report

B08901051 電機四 王濰紳

## 1）

### Experimental Result

| circuit number | # test vector | # gates | # total faults | # detected faults | # undetected faults | fault coverage |
| :----: | :----: | :----: | :----: | :----: | :----: | :----: |
| C499  | 66  | 554  | 2390  | 2263  | 27   | 94.69% |
| C1355 | 63  | 554  | 2726  | 1702  | 1024 | 62.44% |
| C6288 | 42  | 4800 | 17376 | 17109 | 267  | 98.46% |
| C7552 | 289 | 5679 | 19456 | 19144 | 312  | 98.40% |

### sim.cpp
The following code implements the logic simulation. 
First, schedule all the changed input wire.
Second, traverse all the wires in `sort_wlist`, schedule the changed wire, and evaluate the wire value,
the `evaluate()` was defined and will set changes to wire.

```c++
  for (int inWireIndex = 0; inWireIndex < ncktin; ++inWireIndex)
  {
    if (cktin[inWireIndex]->is_changed())
    {
      cktin[inWireIndex]->remove_changed();
      cktin[inWireIndex]->set_scheduled();
    }
  }
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
```

### faultsim.cpp

The folloing code check if the fault is being detected by the currently examined wire.
```c++
  unsigned int detected = 0;
  if (w->is_output())
  {
    detected = w->wire_value_f ^ w->wire_value_g;
    for (int faultIndex = 0; faultIndex < num_of_fault; ++faultIndex)
    {
      if ((detected & Mask[faultIndex]) != 0 
      && (w->wire_value_g & Mask[faultIndex]) != Unknown[faultIndex] 
      && (w->wire_value_f & Mask[faultIndex]) != Unknown[faultIndex])
      {
        simulated_fault_list[faultIndex]->detect = TRUE;
      }
    }
  }
  w->wire_value_f = w->wire_value_g;
```

Update a wire value and set fault, schedule all the
wire's fanout
```c++
  this->combine(w, new_value);
  w->wire_value_f = new_value;
  if (!(w->is_faulty()))
  {
    w->set_faulty();
    wlist_faulty.push_front(w);
  }
  for (int gateIndex = 0; gateIndex < w->onode.size(); ++gateIndex)
  {
    if (w->onode[gateIndex]->type != OUTPUT)
    {
      for (int wireIndex = 0; wireIndex < w->onode[gateIndex]->owire.size(); ++wireIndex)
      {
        w->onode[gateIndex]->owire[wireIndex]->set_scheduled();
      }
    }
  }
```

The following code inject fault according to the fault type.
```c++
	if (fault_type == STUCK0)
	{
		faulty_wire->wire_value_f &= ~Mask[bit_position];
	}
	else if (fault_type == STUCK1)
	{
		faulty_wire->wire_value_f |= Mask[bit_position];
	}
	faulty_wire->inject_fault_at(bit_position);
```

## 2）
### faultsim.cpp
The following code determined a fault detected only after the number set by ndet.
```c++
  for (int faultIndex = 0; faultIndex < num_of_fault; ++faultIndex)
  {
    if (simulated_fault_list[faultIndex]->detect == TRUE)
    {
      simulated_fault_list[faultIndex]->detected_time++;
      if (simulated_fault_list[faultIndex]->detected_time < this->detected_num)
      {
        simulated_fault_list[faultIndex]->detect = FALSE;
      }
    }
  }
```

The following two block of code make sure the edge case where the fault is at output also need to be detected at least N times set by this tool's user.
```c++
  if ((f->node->type == OUTPUT) ||
      (f->io == GO && sort_wlist[f->to_swlist]->is_output()))
  {
    f->detected_time++;
    if (f->detected_time >= this->detected_num)
    {
      f->detect = TRUE;
    }
  }
```

```c++
  if (faulty_wire->is_output())
  {
    f->detected_time++;
    if (f->detected_time >= this->detected_num)
    {
      f->detect = TRUE;
    }
  }
```