


def f( x0, x1, y0, y1, x ):
    return ( ( y0 * ( x1 - x ) ) + ( y1 * ( x - x0 ) ) ) / (x1 - x0)
    
def f2( x0, x1, y0, y1, x ):
    return y0 + ( x - x0 ) * ( y1 - y0 ) / ( x1 - x0 )
    
def fx( x ):
    return f( 1858, 313, 26000, 254000, x )
    
def fy( x ):
    return f( 282, 65, 13000, 40000, x )
    
print( 1858, fx( 1858 ))
print( 313, fx( 313 ))
print( 282, fy( 282 ))
print( 65, fy( 65 ))

print( fx( 0 ), fx( 1 ) - fx( 0 ) )
print( fy( 0 ), fy( 1 ) - fy( 0 ) )  