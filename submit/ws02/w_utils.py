def get_square_from_float( coords ):
    x = int( round( coords[0], 0 ) )
    y = int( round( coords[1], 0 ) )
    return [ x, y ]


def find_intersecting_squares( start, end ):
    startf = [ float(x) for x in start ]
    endf = [ float(x) for x in end ]
    y1 = startf[1]
    y2 = endf[1]
    x1 = startf[0]
    x2 = endf[0]
    
    #current_square = get_square_from_float( startf )
    end_square = get_square_from_float( endf )
    endx = end_square[0]
    endy = end_square[1]

    start_square = get_square_from_float( startf )
    startx = start_square[0]
    starty = start_square[1]

    intersecting_squares = []
    if startx == endx:
        y = starty + 1
        while y < endy:
            intersecting_squares.append( [ startx, y ] )
            y += 1
    elif startx < endx:
        x = startx + 1
        while x < endx:
            y = int( float( ( endy - starty ) ) / ( endx - startx ) * ( float(x) - startx ) + starty )
            intersecting_squares.append( [ x, y ] )
            x += 1
    else:
        x = endx + 1
        while x < startx:
            y = int( float( ( endy - starty ) ) / ( endx - startx ) * ( float(x) - startx ) + starty )
            intersecting_squares.append( [ x, y ] )
            x += 1

    return intersecting_squares

def check_express( current_pos, end, map_list ):
    # check if a straight line can go from there to the end
    candidates_to_check = find_intersecting_squares( current_pos, end )
    for [ x, y ] in candidates_to_check:
        if map_list[x][y] == "B" or map_list[x][y] == "#":
            return False
    
    return True
