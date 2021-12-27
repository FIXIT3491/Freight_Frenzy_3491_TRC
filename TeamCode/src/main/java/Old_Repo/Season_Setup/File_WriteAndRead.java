/* Copied from "https://www.w3schools.com/java/java_files_create.asp"
 *
 * Modified by team FIX IT 3491.
 */

package Old_Repo.Season_Setup;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;


public class File_WriteAndRead extends Ducky {

    /**
     * Writing to a file in the Control Hub
     * @param textToWrite Parameter for what text is written
     */
    public static void writeToFile(String textToWrite) {
        File myFileName = AppUtil.getInstance().getSettingsFile("robotData.txt");
        ReadWriteFile.writeFile(myFileName, textToWrite);
    }

    /**
     * Reading what is on the file in the Control Hub
     */
    public static void readFromFile() {
        File myFileName = AppUtil.getInstance().getSettingsFile("robotData.txt");
        alliance = ReadWriteFile.readFile(myFileName);
    }
}