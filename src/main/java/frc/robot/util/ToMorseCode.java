package frc.robot.util;
import java.util.ArrayList;

public class ToMorseCode {
    public enum MorseCode {
        A(".-"), B("-..."), C("-.-."), D("-.."), E("."), F("..-."), G("--."), H("...."), 
        I(".."), J(".---"), K("-.-"), L(".-.."), M("--"), N("-."), O("---"), P(".--."), 
        Q("--.-"), R(".-."), S("..."), T("-"), U("..-"), V("...-"), W(".--"), X("-..-"), 
        Y("-.--"), Z("--.."), Space("....");
    
        MorseCode(String code) {
        }
    }

    public static ArrayList<MorseCode> toMorseCode(String phrase) {
        phrase.toUpperCase();
        ArrayList<MorseCode> arr = new ArrayList<>();
        for(int i = 0; i < phrase.length(); i++) {
            if(phrase.charAt(i) == ' ') {
                arr.add(MorseCode.Space);
            }
            char cur = phrase.charAt(i);
            String temp = String.valueOf(cur);
            arr.add(MorseCode.valueOf(temp));
        }
        return arr;
    }
}