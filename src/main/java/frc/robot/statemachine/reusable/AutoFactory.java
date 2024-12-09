package frc.robot.statemachine.reusable;

import java.io.FileInputStream;
import java.io.FileNotFoundException;

import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj.Filesystem;

public class AutoFactory {
    public static State loadAuto(StateMachineBase stateMachine, String jsonPath) throws FileNotFoundException {
        String path = Filesystem.getDeployDirectory() + "/autos/" + jsonPath;
        FileInputStream input = new FileInputStream(path);
        JSONTokener tokener = new JSONTokener(input);
        State autoState = parseStateObj(stateMachine, new JSONObject(tokener));
        // Recursively create transitions

        return autoState;
    }

    private static State parseStateObj(StateMachineBase stateMachine, JSONObject state) {
        State s = new State(stateMachine, state.getJSONObject("parameters")) {
            {
                name = state.getString("name");
            }
        };

        if (state.has("children")) {
            for (Object child : state.getJSONArray("children")) {
                s.children.add(parseStateObj(stateMachine, (JSONObject) child));
            }
        }

        return s;
    }

    private static void parseTransitions(StateMachineBase stateMachine, State state, JSONObject transitions) {

    }
}
