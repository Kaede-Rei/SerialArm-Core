# Interaction Capability

Interaction is the robot-level compliance capability of SerialArm-Core

The module is divided into three layers

```
interaction/

├── estimators
│   External torque estimation

├── controllers
│   Compliance controllers

└── runtime
    Runtime orchestration and state exchange
```

The core library provides how the robot reacts to interaction forces

It does not define task constraints or application objectives

Task-level behaviors such as keeping a picking direction, insertion constraints, or manipulation strategies belong to application layers

## State Interface

Runtime interaction output is exchanged through `InteractionState`

It contains:

- estimated external joint torque
- admittance position correction
- admittance velocity correction
- validity state

