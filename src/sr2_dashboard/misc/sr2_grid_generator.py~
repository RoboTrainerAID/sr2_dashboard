class SR2GridGenerator:
  '''
  Returns the dimensions of a grid layout for the distribution of elements of a given list
  '''
  @staticmethod
  def isqrt(n):
    '''
    Returns the integer square root of a given number using Newton's method (so called integer square root)
    '''
    x = n
    y = (x + 1) // 2
    while y < x:
      x = y
      y = (x + n // x) // 2
    return x

  @staticmethod
  def get_dim(list_len):
    '''
    Splits a list in a square-matrix like structure. Used for creation of grid layouts
    Note: the dimensions expand horizontally first and then vertically
    :param list_len: a number that is used to generate the dimensions of the grid
    '''
    col = SR2GridGenerator.isqrt(list_len)

    # Special cases
    # 1x1 for a list with a single element
    if list_len == 1: return (1, 1)

    # Empty list
    if list_len == 0: return (0, 0)

    # Perfect sqrt numbers (4, 9, 16, 25, ...) produce a square matrix without empty cells
    if col**2 == list_len: return (col, col)
    # All other cases
    # Following is suitable for displays with height < width
    # It can easily be reversed so that it can handle height > width
    elif col**2 < list_len <= col*(col+1): return (col, col+1)
    elif col*(col+1) < list_len <= (col+1)**2: return (col+1, col+1)
