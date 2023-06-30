using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CanvasController : MonoBehaviour
{
    public GameObject AICharacter;

    private CharacterNavigation characterNavi;
    private GameObject startHint;
    private GameObject catchTheRobotHint;

    // Start is called before the first frame update
    void Start()
    {
        characterNavi = AICharacter.GetComponent<CharacterNavigation>();
        startHint = GameObject.Find("StartHint");
        catchTheRobotHint = GameObject.Find("CatchTheRobotHint");
        catchTheRobotHint.SetActive(false);
    }

    // Update is called once per frame
    void Update()
    {
        if (characterNavi.GetMoveEnabled())
        {
            startHint.SetActive(false);
        }

        if (characterNavi.GetMoveEnabled() && !characterNavi.GetShouldMove())
        {
            catchTheRobotHint.SetActive(true);
        }
        else
        {
            catchTheRobotHint.SetActive(false);
        }
    }
}
