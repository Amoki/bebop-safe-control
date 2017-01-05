

class Order(object):
    def __init__(self, twist):
        self.linear = twist.linear
        self.angular = twist.angular

    def transform_to_distance_twist(self):
        '''
        Transform the order message (with deltas value in axes) into expected
        tranlations in order to project the futur postion in a map
        '''
        pass

    def transform_to_bebop_twist(self):
        '''
        Transform the order message into bebop compatible twist
        '''
        pass
