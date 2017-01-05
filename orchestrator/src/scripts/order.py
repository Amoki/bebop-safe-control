

class Order(object):
    def __init__(self, twist):
        self.linear = twist.linear
        self.angular = twist.angular

    def transform_to_distance_twist(self):
        '''
        Transform the order message (with deltas value in axes) into expected
        tranlations in order to project the futur postion in a map
        '''
        dis_to_travel.x = self.linear.x * 5
        dis_to_travel.y = self.linear.y * 5
        dis_to_travel.z = self.linear.z * 5

        printf("Order distance to travel is [%s;%s;%s]\n" % (
            self.linear.x,
            self.linear.y,
            self.linear.z)
        )

        return dis_to_travel

    def transform_to_bebop_twist(self):
        '''
        Turn the order message to bebop compatible twist message
        
        linear.x  (+)      Translate forward
                  (-)      Translate backward
        linear.y  (+)      Translate to left
                  (-)      Translate to right
        linear.z  (+)      Ascend
                  (-)      Descend
        
        angular.z (+)      Rotate counter clockwise
                  (-)      Rotate clockwise

        Acceptable range for all fields are [-1;1]
        '''
        speed = 0,2

        if self.linear.y>0:
            print("Drone order is to go left\n")
            y = speed
        elif self.linear.y<0:
            print("Drone order is to go right\n")
            y = -speed

        if self.linear.x>0:
            print("Drone order is to go forward\n")
            x = speed
        elif self.linear.x<0:
            print("Drone order is to go backward\n")
            x = -speed

        if self.linear.z>0:
            print("Drone order is to go up\n")
            z = speed
        elif self.linear.z<0:
            print("Drone order is to go down\n")
            z = -speed


        print("/n")

        linear_vector = Vector3(
            x = x, #left/right
            y = y, #forward/rearward
            z = y #upward/downward
        )

        angular_vector = Vector3(
            x=0,
            y=0,
            z=0
        )

        bebop_twist = twist(linear=linear_vector, angular=angular_vector)

        return bebop_twist