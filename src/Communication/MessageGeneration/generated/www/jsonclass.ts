/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2018-11-12 06:50:47.053470***/
export const JSON_Diagnostic_ID: number = 43794;
export const JSON_Device_ID: number = 43795;
export const JSON_Arm_Status_ID: number = 43824;
export interface Diagnostic {
	rx_count: number;
	DeviceName: string;
	Node_Name: string;
	System: number;
	SubSystem: number;
	Component: number;
	Diagnostic_Type: number;
	Level: number;
	Diagnostic_Message: number;
	Description: string;
}
export class DiagnosticService implements Diagnostic {
	private _rx_count: number;
	get rx_count(): number {
		return this._rx_count;
	}
	set rx_count(v: number) {
		this._rx_count = v;
	}
	private _DeviceName: string;
	private _Node_Name: string;
	private _System: number;
	private _SubSystem: number;
	private _Component: number;
	private _Diagnostic_Type: number;
	private _Level: number;
	private _Diagnostic_Message: number;
	private _Description: string;
	get DeviceName(): string {
		return this._DeviceName;
	}
	set DeviceName(v: string) {
		this._DeviceName = v;
	}
	get Node_Name(): string {
		return this._Node_Name;
	}
	set Node_Name(v: string) {
		this._Node_Name = v;
	}
	get System(): number {
		return this._System;
	}
	set System(v: number) {
		this._System = v;
	}
	get SubSystem(): number {
		return this._SubSystem;
	}
	set SubSystem(v: number) {
		this._SubSystem = v;
	}
	get Component(): number {
		return this._Component;
	}
	set Component(v: number) {
		this._Component = v;
	}
	get Diagnostic_Type(): number {
		return this._Diagnostic_Type;
	}
	set Diagnostic_Type(v: number) {
		this._Diagnostic_Type = v;
	}
	get Level(): number {
		return this._Level;
	}
	set Level(v: number) {
		this._Level = v;
	}
	get Diagnostic_Message(): number {
		return this._Diagnostic_Message;
	}
	set Diagnostic_Message(v: number) {
		this._Diagnostic_Message = v;
	}
	get Description(): string {
		return this._Description;
	}
	set Description(v: string) {
		this._Description = v;
	}
}
export interface Device {
	rx_count: number;
	DeviceParent: string;
	PartNumber: string;
	DeviceName: string;
	DeviceType: string;
	PrimaryIP: string;
	Architecture: string;
	ID: number;
	Capabilities: string[];
	BoardCount: number;
	HatCount: number;
	ShieldCount: number;
	SensorCount: number;
}
export class DeviceService implements Device {
	private _rx_count: number;
	get rx_count(): number {
		return this._rx_count;
	}
	set rx_count(v: number) {
		this._rx_count = v;
	}
	private _DeviceParent: string;
	private _PartNumber: string;
	private _DeviceName: string;
	private _DeviceType: string;
	private _PrimaryIP: string;
	private _Architecture: string;
	private _ID: number;
	private _Capabilities: string[];
	private _BoardCount: number;
	private _HatCount: number;
	private _ShieldCount: number;
	private _SensorCount: number;
	get DeviceParent(): string {
		return this._DeviceParent;
	}
	set DeviceParent(v: string) {
		this._DeviceParent = v;
	}
	get PartNumber(): string {
		return this._PartNumber;
	}
	set PartNumber(v: string) {
		this._PartNumber = v;
	}
	get DeviceName(): string {
		return this._DeviceName;
	}
	set DeviceName(v: string) {
		this._DeviceName = v;
	}
	get DeviceType(): string {
		return this._DeviceType;
	}
	set DeviceType(v: string) {
		this._DeviceType = v;
	}
	get PrimaryIP(): string {
		return this._PrimaryIP;
	}
	set PrimaryIP(v: string) {
		this._PrimaryIP = v;
	}
	get Architecture(): string {
		return this._Architecture;
	}
	set Architecture(v: string) {
		this._Architecture = v;
	}
	get ID(): number {
		return this._ID;
	}
	set ID(v: number) {
		this._ID = v;
	}
	get Capabilities(): string[] {
		return this._Capabilities;
	}
	set Capabilities(v: string[]) {
		this._Capabilities = v;
	}
	get BoardCount(): number {
		return this._BoardCount;
	}
	set BoardCount(v: number) {
		this._BoardCount = v;
	}
	get HatCount(): number {
		return this._HatCount;
	}
	set HatCount(v: number) {
		this._HatCount = v;
	}
	get ShieldCount(): number {
		return this._ShieldCount;
	}
	set ShieldCount(v: number) {
		this._ShieldCount = v;
	}
	get SensorCount(): number {
		return this._SensorCount;
	}
	set SensorCount(v: number) {
		this._SensorCount = v;
	}
}
export interface Arm_Status {
	rx_count: number;
	armed_status: number;
}
export class Arm_StatusService implements Arm_Status {
	private _rx_count: number;
	get rx_count(): number {
		return this._rx_count;
	}
	set rx_count(v: number) {
		this._rx_count = v;
	}
	private _armed_status: number;
	get armed_status(): number {
		return this._armed_status;
	}
	set armed_status(v: number) {
		this._armed_status = v;
	}
}
