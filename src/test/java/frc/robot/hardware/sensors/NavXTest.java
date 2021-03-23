package frc.robot.hardware.sensors;


import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.mockito.Mockito.mock;

import org.mockito.Mockito;

public class NavXTest {
    
    @Test
    public void testGetAngle(){
        //Expected output, Current Total Angle, Last Total Angle, Current rate of robot rotation
        NavXTestCases[] testCases = {
            new NavXTestCases(-75, 120, 45, -46),
            new NavXTestCases(-305, 733, 503, -35),
            new NavXTestCases(898, 8437, 7234, 54),
            new NavXTestCases(1390, 9013, 8521, 56),
            new NavXTestCases(1390, 9013, 9013, 56),
            new NavXTestCases(-6597, 17_000, 9013, -56),
        };
        NavX navX = Mockito.spy(NavX.get());
        

        assertNotNull("NavX mock was null", navX);

        // Run through each of the test cases feeding in our data in place of the Total angle, current rate and last angle
        for (NavXTestCases testCase : testCases) {
            
            Mockito.when(navX.getAngle()).thenReturn(testCase.currentAngle);
            Mockito.when(navX.getRate()).thenReturn(testCase.currentRate);
            Mockito.when(navX.getLastHeading()).thenReturn(testCase.lastAngle);

            assertEquals(testCase.expectedOutput, navX.getAngle(), 0.01);
        }
        
        
    }
}
