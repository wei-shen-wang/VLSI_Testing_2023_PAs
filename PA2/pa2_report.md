# PA2 Report

B08901051 電機四 王濰紳


## Experimental Result

| circuit number | # gates | # total faults | # detected faults | # undetected faults | fault coverage | # test vector | # runtime |
| :----: | :----: | :----: | :----: | :----: | :----: | :----: | :----: |
| C432  | 245  | 1110  | 149   | 961  | 13.42% | 20  | 0.0s |
| C499  | 554  | 2390  | 2280  | 110  | 95.40% | 74  | 0.1s |
| C880  | 545  | 2104  | 1254  | 850  | 59.60% | 62  | 0.1s |
| C1355 | 554  | 2726  | 1702  | 1024 | 62.44% | 63  | 0.4s |
| C2670 | 1785 | 6520  | 6278  | 242  | 96.29% | 156 | 0.2s |
| C3540 | 2082 | 7910  | 2424  | 5486 | 30.64% | 95  | 3.0s |
| C6288 | 4800 | 17376 | 17109 | 267  | 98.46% | 47  | 0.4s |
| C7552 | 5679 | 19456 | 19154 | 302  | 98.40% | 267 | 1.2s |

## podem.cpp

### TODO 1
The following code backtrace from current `object_wire` to the Primary Input and find assign PI with heuristics. If `NOT`, `NOR`, `NAND` the `object_level` will be inverted, if current gate is imply gate choose the hardest gate input to backtrace, if current gate is decision gate choose th easiest gate input to backtrace.
```c++
switch (object_wire->inode.front()->type)
{
  case BUF:
    new_object_level = object_level;
    new_object_wire = find_easiest_control(object_wire->inode.front());
    break;
  case NOT:
    new_object_level = object_level ^ 1;
    new_object_wire = find_easiest_control(object_wire->inode.front());
    break;
  case OR:
    new_object_level = object_level;
    if (object_level == 1)
    {
      new_object_wire = find_easiest_control(object_wire->inode.front());
    }
    else
    {
      new_object_wire = find_hardest_control(object_wire->inode.front());
    }
    break;
  case NAND:
    new_object_level = object_level ^ 1;
    if (object_level == 1)
    {
      new_object_wire = find_easiest_control(object_wire->inode.front());
    }
    else
    {
      new_object_wire = find_hardest_control(object_wire->inode.front());
    }
    break;
  case AND:
    new_object_level = object_level;
    if (object_level == 1)
    {
      new_object_wire = find_hardest_control(object_wire->inode.front());
    }
    else
    {
      new_object_wire = find_easiest_control(object_wire->inode.front());
    }
    break;
  case NOR:
    new_object_level = object_level ^ 1;
    if (object_level == 1)
    {
      new_object_wire = find_hardest_control(object_wire->inode.front());
    }
    else
    {
      new_object_wire = find_easiest_control(object_wire->inode.front());
    }
    break;
}
```
### TODO 2
The folloing code trace X path by DFS searching (recursive calling it self) for existing x path from current wire to Primary Output.

**Bonus Implementation Explanation**
Notice that I added an attribute `xPathStatus` to the class `WIRE` for checking the current `xPathStatus`, the `xPathStatus` is originally all set to -1(unknown), after being traced by `trace_unknown_path()` the status will be changed to 1 if there exists any X path to Primary Output, 0 if there exists no X path to Primary Output. After pattern generation for a fault, all `xPathStatus` will be reset to -1 by the *Inserted Code outside TODO* in the function `test_possible()`. Due to this recording every path won't be repeatedly traversed twice during pattern generation for a single fault. Therefore the time complexity of `trace_unknown_path()` with the bonus implementation is $O(n)$ time complexity trading of with extra $O(n)$ space complexity.

If the bonus is not implemented the original time complexity is $O(a^n)$ where $a$ could be any positive integer larger than 1.

```c++
if (w->is_output() || w->xPathStatus == 1)
{
  w->xPathStatus = 1;
  return true;
}
if (w->xPathStatus == 0)
{
  return false;
}
for (int i = 0; i < w->onode.size(); i++)
{
  if (w->onode[i]->owire.front()->value == U)
    if (trace_unknown_path(w->onode[i]->owire.front()))
    {
      return true;
    }
}
w->xPathStatus = 0;
return false;
```

**Inserted Code outside TODO**
```c++
for (int i = 0; i < this->sort_wlist.size(); i++)
{
  sort_wlist[i]->xPathStatus = -1;
}
```

The following code do backward imply to try set a Primary Input value with objective being setting the value of current gate under test to the opposite of its stuck at value.
```c++
int desiredvalue;
if (fault->fault_type == STUCK0)
{
  desiredvalue = 1;
}
else if (fault->fault_type == STUCK1)
{
  desiredvalue = 0;
}
switch (backward_imply(w, desiredvalue))
{
  case TRUE:
    pi_is_reach = TRUE;
    break;
  case CONFLICT:
    return CONFLICT;
  case FALSE:
    break;
}
```