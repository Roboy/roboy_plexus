#!/bin/bash
echo setpoint $1
rostopic pub /roboy/middleware/HandCommand roboy_middleware_msgs/HandCommand "id: 0
setPoint: [$1,$1,$1,$1,$1,$1,$1,$1,$1,$1,$1,$1,$1,$1,$1,$1,$1,$1,$1,$1]  
motorid: [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]"
