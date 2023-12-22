
class Number(object):
    def __init__(self, number):
        self.number = number

    def get_number(self):
        return self.number

class Name(object):
    def __init__(self, name):
        self.name = name
    
    def get_name(self):
        return self.name

class Move(object):
    def __init__(self, move_x, move_y):
        self.arguments = [None, None]
        self.arguments[0] = Number(move_x)
        self.arguments[1] = Number(move_y)
    
    def get_value(self):
        return (self.arguments[0].get_number(), self.arguments[1].get_number())


class Action(object):
    def __init__(self, move_x, move_y, name = 'move'):
        
        self.arguments = [None, None]
        self.arguments[0] = Name(name)
        self.arguments[1] = Move(move_x, move_y)
        
    def get_name(self):
        return self.arguments[0].get_name()

    def get_value(self):
        return self.arguments[1].get_value()

    ''''
    def set(self, move_x, move_y, name = 'move'):
        self.arguments[0].name = name
        self.arguments[1].arguments[0].number = move_x
        self.arguments[1].arguments[1].number = move_y
    '''
