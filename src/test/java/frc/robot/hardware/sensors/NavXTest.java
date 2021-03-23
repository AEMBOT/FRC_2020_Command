package frc.robot.hardware.sensors;


import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.mockito.Mockito.mock;

import org.mockito.Mockito;

public class NavXTest {
    
    @Test
    public void testGetAngle(){
        NavXTestCases[] testCases = {
            new NavXTestCases(-75, 120, 45, -46),
            new NavXTestCases(-305, 733, 503, -35),
            new NavXTestCases(898, 8437, 7234, 54),
            new NavXTestCases(1390, 9013, 8521, 56)
        };
        NavX navX = mock(NavX.class);

        assertNotNull("NavX mock was null", navX);

        for (NavXTestCases testCase : testCases) {
            Mockito.when(navX.getTotalAngleDeg()).thenReturn(testCase.currentAngle);
            Mockito.when(navX.getRate()).thenReturn(testCase.currentRate);
            Mockito.when(navX.getLastHeading()).thenReturn(testCase.lastAngle);
            
            assertEquals(testCase.expectedOutput, navX.getAngle(), 0.01);
        }
        
        
    }
}
