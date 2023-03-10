# Tabula Messages

## Parameter Tree

The parameter tree dictates:
- how a particular parameter type (i.e., Text Content) can be changed
- how changing a particular parameter type will affect other parameters down the tree

```
-- World (*)
    |
    |-- Program
        |
        |-- Recording (d)
            |
            |-- Text Content (*db)
            |   |
            |   |-- Label Interval (*d)
            |       |
            |       |-- Label (*d)
            |
            |-- User Sequence (+db)
                |
                |-- Plan (d)
                    |
                    |-- Waypoint (pd)
                        |
                        |-- Task Subsequence (qd)
                            |
                            |-- Command (rd)
                            |
                            |-- Conditional (rd)
                            |
                            |-- Jump Point (rd)


KEY:
(*)  --  Parameter is directly editable.
(+)  --  Paths between waypoints are directly editable.
(d)  --  Deletable.
(b)  --  Redoable (equivalent to deleting and re-recording).
(q)  --  Rearrangeable. 
(p)  --  Confirm (lock), deny (prevent) from adding to user seq., or unrestricted.
(r)  --  Confirm (lock), deny (prevent) from adding to task seq., or unrestricted.
```


## Message Structure

The structure of the json string within ```TabulaUpdate.msg```:

```
{
  "world":
  {
    "regions":
    {
      REGION_NAME: 
      {
        "name": REGION_NAME (string),
        "objects":
        [
          "name": OBJECT_NAME (string)
          "objects": [ ... ]
        ]
      }
    }
  }

  "program":
  {
    "recordings":
    [
      {
        "_id": RECORDING_ID (int),
        "text":
        {
          "content": CONTENT (string),
          "label_intervals": 
          [
            {
              "start": START_POS (int),
              "end": END_POS (int),
              "label":
              {
                "cmd": ACTION_NAME (str)
                "_type": ACTION_TYPE (str)
                "args":
                [
                  {
                    "argname": ARG_NAME (string),
                    "argval": ARG_VAL (string)
                  },
                  ...
                ]
              }
            }
            ...
          ]
        }
        "sketch":
        {
          "user_sequence":
          {
            "data":
            [
              {
                "_id": ACTION_ID (int),
                "cmd": ACTION_NAME (string),
                "_type": CATEGORICAL ("command", "conditional")
                "args":
                [
                  {
                    "argname": ARG_NAME (string),
                    "argval": ARG_VAL (string)
                  },
                  ...
                ]
              }
              ...
            ]
            "plan":
            {
              "waypoints":
              [
                {
                  "_id": WAYPOINT_ID (int),
                  "x": X_POS (int),
                  "y": Y_POS (int),
                  "task_subsequence":
                  [
                    {
                      "_id": ACTION_ID (int),
                      "cmd": ACTION_NAME (string),
                      "_type": CATEGORICAL ("command", "conditional")
                      "args":
                      [
                        {
                          "argname": ARG_NAME (string),
                          "argval": ARG_VAL (string)
                        },
                        ...
                      ]
                    }
                  ]
                }
                ...
              ]
            }
          }
        }
      }
      ...
    ]
  }
}
```

TabulaMsg also contains an update `type` field. The types of updates are:

```
0: no update
1: new recording
```