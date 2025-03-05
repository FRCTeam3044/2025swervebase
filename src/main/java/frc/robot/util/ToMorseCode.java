package frc.robot.util;
import java.util.ArrayList;

public class ToMorseCode {

    public enum MorseCode {
        A(".-"), B("-..."), C("-.-."), D("-.."), E("."), F("..-."), G("--."), H("...."), 
        I(".."), J(".---"), K("-.-"), L(".-.."), M("--"), N("-."), O("---"), P(".--."), 
        Q("--.-"), R(".-."), S("..."), T("-"), U("..-"), V("...-"), W(".--"), X("-..-"), 
        Y("-.--"), Z("--.."), Space(" "), Zero("-----"), One(".----"), Two("..---"), 
        Three("...--"), Four("....-"), Five("....."), Six("-...."), Seven("--..."), 
        Eight("---.."), Nine("----.");

        private final String code;

        MorseCode(String code) {
            this.code = code;
        }

        public String getCode() {
            return this.code;
        }
    }

    public static String toMorseCode(String phrase) {
        phrase = phrase.toUpperCase(); // Convert to uppercase to ensure correct enum matching
        ArrayList<String> morseList = new ArrayList<>();

        for (int i = 0; i < phrase.length(); i++) {
            char cur = phrase.charAt(i);
            if (cur == ' ') {
                morseList.add(MorseCode.Space.getCode()); 
            } 
            else if(cur == '0') {
                morseList.add(MorseCode.Zero.getCode());
            }
            else if(cur == '1') {
                morseList.add(MorseCode.One.getCode());
            }
            else if(cur == '2') {
                morseList.add(MorseCode.Two.getCode());
            }
            else if(cur == '3') {
                morseList.add(MorseCode.Three.getCode());
            }
            else if(cur == '4') {
                morseList.add(MorseCode.Four.getCode());
            }
            else if(cur == '5') {
                morseList.add(MorseCode.Five.getCode());
            }
            else if(cur == '6') {
                morseList.add(MorseCode.Six.getCode());
            }
            else if(cur == '7') {
                morseList.add(MorseCode.Seven.getCode());
            }
            else if(cur == '8') {
                morseList.add(MorseCode.Eight.getCode());
            }
            else if(cur == '9') {
                morseList.add(MorseCode.Nine.getCode());
            } 
            else {
                morseList.add(MorseCode.valueOf(String.valueOf(cur)).getCode());
            }
        }

        return String.join(" ", morseList);  // Join the Morse codes with spaces
    }
    
}