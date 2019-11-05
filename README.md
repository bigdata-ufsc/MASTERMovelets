# MASTERMovelets_Optimizations
This is a project with the MASTERMovelets implementation, with three options of optimizations:
-MASTERMovelet-Log: limiting the movelets size to the ln size of the trajectory
-MASTERMovelet-Pivots: it limit the movelets search space to the points that are neighbour of well qualified movelets of size one.
-SUPER-MASTERMovelet: a supervised approach that identifies the better regions for finding movelets.

To run the MASTERMovelets, MASTERMovelet-Log and MASTERMovelet-Pivots, you need to run the MoveletsRunUnit.java class, and use the following entries as example:

MASTERMovelets:
-curpath "$BASIC_PATH" 
-respath "$BASIC_PATH" 
-descfile "$DESC_MASTER"  
-nt 8 -ed true -ms 1 -cache true -output discrete -samples 1 -sampleSize 0.5 -medium "none" -output "discrete" -lowm "false"

MASTERMovelet-Log:
-curpath "$BASIC_PATH" 
-respath "$LOG_PATH" 
-descfile "$DESC_MASTER"  
-nt 8 -ed true -ms 1 -Ms -3 -cache true -output discrete -samples 1 -sampleSize 0.5 -medium "none" -output "discrete" -lowm "false"

MASTERMovelet-Pivots:
-curpath "$BASIC_PATH" 
-respath "$BASIC_PATH" 
-descfile "$DESC_MASTER"  
-nt 8 -ed true -ms 1 -cache true -output discrete -samples 1 -sampleSize 0.5 -medium "none" -output "discrete" -lowm "false" -pvt true -lp false -pp 10 -op false 

To run the -SUPER-MASTERMovelet, you need to run the MoveletsRunUnit_Supervised.java class, and use the following entries as example:

-SUPER-MASTERMovelet:
-curpath "$BASIC_PATH" 
-respath "$BASIC_PATH" 
-descfile "$DESC_MASTER"  
-nt 8 -ed true -ms 1 -cache true -output discrete -samples 1 -sampleSize 0.5 -medium "none" -output "discrete" -lowm "false"

-SUPER-MASTERMovelet-Log:
-curpath "$BASIC_PATH" 
-respath "$BASIC_PATH" 
-descfile "$DESC_MASTER"  
-nt 8 -ed true -ms 1 -Ms -3 -cache true -output discrete -samples 1 -sampleSize 0.5 -medium "none" -output "discrete" -lowm "false"

To include the attribute limit, you only need to add "-Al true" to the input
