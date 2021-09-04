{
    "config": [
        {
            "filterTyp": "EKF",   # Type "EKF" or "UKF" (attention UKF has bug)
            "PredictFrequency": 50, # Frquency of Prediction
            "lengthDatabag": 50, # Length of Databag for PastTimeCorrection
            "RosRun": true # Flag if Ros is runing
            "MeasErrLoc": 0, # white noise PreInput (location)
            "MeasErrScale": 0 # white noise PreInput (scale)
        },

        {
            "type": "UKF",
            "settings": {
                "beta": 2,
                "alpha": 0.001,
                "k": 0,
                "ProcessNoiseRt": [[0.3,0,0],[0,0.3,0],[0,0,0.3]],
                "CovarianzMatrixOfRt": 1,
                "InitState": [1, 0, 0.5], #init State und Position von Agent müssen zwingend übereinstimmen!!!!
                "InitCovar": [[1,0,0],[0,1,0],[0,0,1]]
            }
        },

        {
            "type": "EKF",
            "settings": {
                "ProcessNoiseRt": [[0.3,0,0],[0,0.3,0],[0,0,0.3]],
                "CovarianzMatrixOfRt": 1,
                "InitState": [1, 0, 0.5], #init State und Position von Agent müssen zwingend übereinstimmen!!!!
                "InitCovar": [[1,0,0],[0,1,0],[0,0,1]]
            }
        }
    ]
}