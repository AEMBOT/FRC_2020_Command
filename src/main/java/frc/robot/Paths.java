package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/***
 * Final class with static path constants
 */
public final class Paths {

    /**
     * Construct the list of waypoints to be used by the barrel run path
     * 
     * @return list of translation points
     */
    public static List<Translation2d> getBarrelPoints() {
        List<Translation2d> waypoints = new ArrayList<Translation2d>();
        // Pull info out of a pre drawn spline
        waypoints.add(new Translation2d(1.9233950087618585, -2.230884041331802));
        waypoints.add(new Translation2d(2.8574654661913104, -2.138057163574838));
        waypoints.add(new Translation2d(3.785734243760952, -2.3063058795093356));
        waypoints.add(new Translation2d(4.18605015408786, -2.7936469877333976));
        waypoints.add(new Translation2d(4.336893830442927, -3.246178016798598));
        waypoints.add(new Translation2d(4.1454383950691875, -3.5362620097891106));
        waypoints.add(new Translation2d(3.6174855278264544, -3.588477128527403));
        waypoints.add(new Translation2d(3.327401534835941, -3.2519796966584082));
        waypoints.add(new Translation2d(3.408625052873285, -2.892275545350171));
        waypoints.add(new Translation2d(3.6871056861441778, -2.5267697141821257));
        waypoints.add(new Translation2d(4.40651398876065, -2.3759260378270586));
        waypoints.add(new Translation2d(5.299972687171429, -2.3411159586681975));
        waypoints.add(new Translation2d(6.489317058432533, -1.8479731705843252));
        waypoints.add(new Translation2d(6.837417850021149, -1.3374253429210226));
        waypoints.add(new Translation2d(6.547333857030635, -1.0357379902108887));
        waypoints.add(new Translation2d(6.0657944286663845, -1.0531430297903195));
        waypoints.add(new Translation2d(5.601660039881564, -1.2678051846032992));
        waypoints.add(new Translation2d(5.6306684391806145, -1.7841546921264124));
        waypoints.add(new Translation2d(5.8105205148347325, -2.3063058795093356));
        waypoints.add(new Translation2d(6.176026346002779, -2.805250347453018));
        waypoints.add(new Translation2d(6.924443047918303, -3.3506082542751825));
        waypoints.add(new Translation2d(7.951340383104719, -3.4376334521723364));
        waypoints.add(new Translation2d(8.311044534412954, -2.915482264789413));
        waypoints.add(new Translation2d(7.957142062964528, -2.3759260378270586));
        waypoints.add(new Translation2d(7.127501843011661, -2.3295125989485768));
        waypoints.add(new Translation2d(6.083199468245815, -2.317909239228956));
        waypoints.add(new Translation2d(4.679192942171732, -1.9756101275001507));
        waypoints.add(new Translation2d(3.2055662577799255, -1.801559731705843));
        return waypoints;
    }

    /**
     * Construct the list of waypoints to be used by the bounce run path
     * 
     * @return list of translation points
     */
    public static List<Translation2d> getBouncePoints() {
        List<Translation2d> waypoints = new ArrayList<Translation2d>();
        waypoints.add(new Translation2d(1.7261378935283096, -2.225082361471992));
        waypoints.add(new Translation2d(2.045230285817874, -1.8363698108647046));
        waypoints.add(new Translation2d(2.2192806816121817, -1.1923833464257656));
        waypoints.add(new Translation2d(2.2947025197897153, -0.6876371986222731));
        waypoints.add(new Translation2d(2.358520998247628, -1.2968135839023505));
        waypoints.add(new Translation2d(2.422339476705541, -1.824766451145084));
        waypoints.add(new Translation2d(2.6892167502568127, -2.515166354462505));
        waypoints.add(new Translation2d(3.072127621004289, -3.228772977219167));
        waypoints.add(new Translation2d(3.4144267327330953, -3.7451224847422804));
        waypoints.add(new Translation2d(4.267273672125203, -3.6638989667049366));
        waypoints.add(new Translation2d(4.568961024835337, -2.9793007432473257));
        waypoints.add(new Translation2d(4.534150945676475, -1.9233950087618585));
        waypoints.add(new Translation2d(4.534150945676475, -1.105358148528612));
        waypoints.add(new Translation2d(4.586366064414768, -0.6644304791830322));
        waypoints.add(new Translation2d(4.673391262311922, -1.4302522206779864));
        waypoints.add(new Translation2d(4.7604164602090755, -2.3005041996495255));
        waypoints.add(new Translation2d(4.876450057405281, -2.985102423107136));
        waypoints.add(new Translation2d(5.189740769835034, -3.634890567405885));
        waypoints.add(new Translation2d(5.8105205148347325, -3.6871056861441778));
        waypoints.add(new Translation2d(6.437101939694241, -3.5130552903498695));
        waypoints.add(new Translation2d(6.703979213245512, -2.6253982717989));
        waypoints.add(new Translation2d(6.791004411142666, -1.9117916490422378));
        waypoints.add(new Translation2d(6.849021209740769, -0.7340506375007554));
        waypoints.add(new Translation2d(6.9302447277781125, -1.28521022418273));
        waypoints.add(new Translation2d(7.429189195721795, -1.981411807359961));

        return waypoints;
    }

    /**
     * Construct the list of waypoints to be used by the bounce run path
     * 
     * @return list of translation points
     */
    public static List<Translation2d> getSlalomPoints() {
        List<Translation2d> waypoints = new ArrayList<Translation2d>();
        waypoints.add(new Translation2d(1.934998368481479, -3.7799325639011414));
        waypoints.add(new Translation2d(2.2366857211916127, -3.3854183334340435));
        waypoints.add(new Translation2d(2.7704402682941565, -2.5209680343223155));
        waypoints.add(new Translation2d(3.1939628980603056, -2.138057163574838));
        waypoints.add(new Translation2d(3.5768737688077823, -1.9291966886216687));
        waypoints.add(new Translation2d(4.133835035349567, -1.778353012266602));
        waypoints.add(new Translation2d(4.696597981751163, -1.7203362136684994));
        waypoints.add(new Translation2d(5.241955888573327, -1.8479731705843252));
        waypoints.add(new Translation2d(5.75250371623663, -2.085842044836546));
        waypoints.add(new Translation2d(6.4254985799746205, -2.5615797933409867));
        waypoints.add(new Translation2d(6.889632968759441, -3.4840468910508187));
        waypoints.add(new Translation2d(7.405982476282554, -3.953982959695449));
        waypoints.add(new Translation2d(8.044167260861682, -3.919172880536588));
        waypoints.add(new Translation2d(8.595326847543657, -3.4666418514713877));
        waypoints.add(new Translation2d(8.757773883618345, -2.8574654661913104));
        waypoints.add(new Translation2d(8.351656293431626, -2.2134790017523716));
        waypoints.add(new Translation2d(7.696066469273067, -2.0394286059580637));
        waypoints.add(new Translation2d(7.156510242310713, -2.1728672427336995));
        waypoints.add(new Translation2d(6.8258144903015285, -2.642803311378331));
        waypoints.add(new Translation2d(6.518325457731584, -3.460840171611577));
        waypoints.add(new Translation2d(6.0251826696477115, -3.8959661610973466));
        waypoints.add(new Translation2d(5.334782766330291, -3.988793038854311));
        waypoints.add(new Translation2d(4.63277950329325, -3.97138799927488));
        waypoints.add(new Translation2d(4.0003963985739315, -3.9249745603963984));
        waypoints.add(new Translation2d(3.4666418514713877, -3.872759441658106));
        waypoints.add(new Translation2d(3.0257141821258076, -3.6987090458637977));
        waypoints.add(new Translation2d(2.503562994742884, -3.29259145567708));
        waypoints.add(new Translation2d(2.022023566378633, -2.7066217898362437));
        waypoints.add(new Translation2d(1.5868975768928635, -2.2598924406308534));

        return waypoints;
    }

    
    /**
     * Construct the list of waypoints to be used by the bounce run path
     * 
     * @return list of translation points
     */
    public static List<Translation2d> getTestPoints() {
        List<Translation2d> waypoints = new ArrayList<Translation2d>();
        waypoints.add(new Translation2d(1, 0));

        return waypoints;
    }

    /**
     * Construct the list of waypoints to be used by the test curve
     * 
     * @return list of translation points
     */
    public static List<Translation2d> getTestCurvePoints() {
        List<Translation2d> waypoints = new ArrayList<Translation2d>();
        waypoints.add(new Translation2d(1, 0.75));

        return waypoints;
    }
}