## Storyline 

<img width="831" alt="one" src="https://user-images.githubusercontent.com/67200542/153248150-ad0ffde0-3107-4858-ab01-3f0d2a53cc9e.PNG">

## The Background
Strawberry harvesting on a commercial scale requires skilled labour. The harvester has to quickly scan the ripe strawberries on a plant and then deftly proceed to pick them, packing the berries in boxes happens at the same time. You can see this process in action in the two videos below:
But did you notice that while the workers are performing the skilled work of picking , since they are paid by the box, they also have to pick the box and run back to a truck to deposit it. This is not good for the overall efficiency of the farm operation and is also toil for the workers, surely there could be a better way.

## Some inferences
Scalability of the cart is an issue because you need one for each row, roughly speaking.
Storage and maintenance both during and apart from the harvest season is an issue, even if they are normal non-smart carts.
Below is a view of how a typical strawberry operation looks like, consider how many carts this will necessitate, even simple low-tech carts soon become a cost and logistics challenge.

<img width="516" alt="one" src="https://user-images.githubusercontent.com/67200542/153268488-9dfb0da4-ec18-4668-913e-c0604adbb6a8.PNG">

## Designing our intervention
Can we improve upon the selected approach?

Two clear areas are the capital outlay and the running costs. For instance, the huge cart cost a whopping $125,000 back in 2004 and incurred about $200 a day in operations and maintenance costs.
A sound approach is repurposing existing tools and implements if possible.
Perhaps sprayer arms, or harvester heads could be repurposed but they are really not designed for the job and unsuitable for load bearing.
We are seeing that multicopters are increasingly being used on farms to perform spraying operations, some organic farms even employ them to spray/deposit beneficial bugs 17!

## Documentation

[Ros Commands](https://subscription.packtpub.com/book/hardware_and_creative/9781782175193/1/ch01lvl1sec15/ros-commands-summary)

[Mavros](http://wiki.ros.org/mavros)

[strawberry stacker](https://github.com/erts-RnD/strawberry_stacker)

[Reference](https://clover.coex.tech/en/mavros.html)

Task 4 
In the single_vehicle_spawn_sdf.launch file by changing the second last line -model $(arg vehicle)_$(arg ID) to -model edrone$(arg ID) has resolved this error and this will also solve the gripper error , everything works fine and i hope this might be the final bug we are facing


