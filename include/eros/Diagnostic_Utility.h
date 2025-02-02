#pragma once
#include <eros/Diagnostic.h>
#include <eros/diagnostic.h>
namespace eros {
eros::diagnostic convert(Diagnostic::DiagnosticDefinition def) {
    eros::diagnostic diag;
    diag.DeviceName = def.device_name;
    diag.NodeName = def.node_name;
    diag.System = (uint8_t)def.system;
    diag.SubSystem = (uint8_t)def.subsystem;
    diag.Component = (uint8_t)def.component;
    diag.DiagnosticType = (uint8_t)def.type;
    diag.DiagnosticMessage = (uint8_t)def.message;
    diag.Level = (uint8_t)def.level;
    diag.Description = def.description;
    return diag;
}

Diagnostic::DiagnosticDefinition convert(eros::diagnostic diag) {
    Diagnostic::DiagnosticDefinition def;
    def.device_name = diag.DeviceName;
    def.node_name = diag.NodeName;
    def.system = (System::MainSystem)diag.System;
    def.subsystem = (System::SubSystem)diag.SubSystem;
    def.component = (System::Component)diag.Component;
    def.type = (Diagnostic::DiagnosticType)diag.DiagnosticType;
    def.message = (Diagnostic::Message)diag.DiagnosticMessage;
    def.level = (Level::Type)diag.Level;
    def.description = diag.Description;
    return def;
}
}  // namespace eros