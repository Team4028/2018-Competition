package org.usfirst.frc.team4028.util;

import java.util.ArrayList;
import java.util.List;

// This is a "data entity" class that hold the data to logged.
// Subsystem classes use the Add method to add data in their UpdateLogData method as Name/Value pairs
// Internally, this class holds the Names & Values in 2 arrays
// Therefore this class does not need to be changed to support addl data to be logged, it grows dynamically
public class LogData {
	// define class level working variables
	private List<String> _names;
	private List<String> _values;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public LogData() {
		_names = new ArrayList<String>();
		_values = new ArrayList<String>();
	}
		
	//============================================================================================
	// Methods follow
	//============================================================================================
	public void AddData(String name, String value) {
		_names.add(name);
		_values.add(value);
	}
	
	// discard any data currently being held
	public void ResetData() {
		_names.clear();
		_values .clear();
		
		//_names = new ArrayList<String>();
		//_values = new ArrayList<String>();	
	}

	// build a TSV (tab separated value) string for the header row
	public String BuildTSVHeader() {
		return BuildTSVString(_names);
	}

	// build a TSV (tab separated value) string for a data row
	public String BuildTSVData() {
		return BuildTSVString(_values);
	}
	
	// build a TSV string from a List<string>
	private String BuildTSVString(List<String> myList) {
		StringBuilder sb = new StringBuilder();
		
		for(String item : myList) {
			// add the item + a tab character
			sb.append(item + "\t");
		}
		
		String lineToWrite = sb.toString();
		
		// remove the trailing tab
		lineToWrite = lineToWrite.substring(0, lineToWrite.length() - 1);
		
		// add trailing crlf
		lineToWrite = lineToWrite + "\r\n";
		
		return lineToWrite;
	}
}