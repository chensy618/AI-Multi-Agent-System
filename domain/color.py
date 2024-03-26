from enum import Enum, unique

@unique
class Color(Enum):
    Blue = 0
    Red = 1
    Cyan = 2
    Purple = 3
    Green = 4
    Orange = 5
    Pink = 6
    Grey = 7
    Lightblue = 8
    Brown = 9
    
    @staticmethod
    def from_string(s: 'str') -> 'Color':
        s = s.lower()
        if s == 'blue':
            return Color.Blue
        elif s == 'red':
            return Color.Red
        elif s == 'cyan':
            return Color.Cyan
        elif s == 'purple':
            return Color.Purple
        elif s == 'green':
            return Color.Green
        elif s == 'orange':
            return Color.Orange
        elif s == 'pink':
            return Color.Pink
        elif s == 'grey':
            return Color.Grey
        elif s == 'lightblue':
            return Color.Lightblue
        elif s == 'brown':
            return Color.Brown
        else:
            return None
