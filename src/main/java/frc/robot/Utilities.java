package frc.robot;

public class Utilities {

    public static double convertReferenceAngleToActualAngle(double referenceAngle, int quadrant) {
     
        double actualAngle = 0;

        switch (quadrant) {
            case 1: 
              actualAngle = referenceAngle;
              break;
            case 2:
               actualAngle =  180 - referenceAngle;
               break;
            case 3:
                actualAngle =  180 + referenceAngle;
                break;
            case 4:
                actualAngle =  360 - referenceAngle;
                break;
            default:
                System.out.println("Error in convertReferenceToActualAngle");
        }
        
        return actualAngle;
    }

}