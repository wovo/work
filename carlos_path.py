import patherror

def carlos_path( cycle ):
    #raise patherror.PathError( "Hi Carlos, time for lunch" )
    return {
        "cycle": cycle, 
        "steps" :[
            {
                "destination": {
                    "x": 1000,
                    "y": 0,
                    "z": 20000
                },
                "speed": {
                    "x": 100,
                    "y": 100,
                    "z": 100
                },
                "spraying" : True
            },
            {
                "destination": {
                    "x": 300000,
                    "y": 15000,
                    "z": 1000
                },
                "speed": {
                    "x": 50,
                    "y": 100,
                    "z": 100
                },
                "spraying" : False
            },
            {
                "destination": {
                    "x": 1000,
                    "y": 40000,
                    "z": 1000
                },
                "speed": {
                    "x": 100,
                    "y": 100,
                    "z": 100
                },
                "spraying" : True
            }
        ]
    }