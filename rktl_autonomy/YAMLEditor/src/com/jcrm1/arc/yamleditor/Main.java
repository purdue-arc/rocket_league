package com.jcrm1.arc.yamleditor;

import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Arrays;
import java.util.regex.Pattern;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.table.DefaultTableModel;
/**
 * Unpack into a table, edit, and repack a YAML-styled 2D array
 * 
 * @author Campbell McClendon
 *
 * @version 0.1
 */
public class Main {
	private static JFrame frame;
	private static JTextField inputField;
	private static JButton updateButton;
	private static final Rectangle scrollPaneBounds = new Rectangle(2, 44, 496, 454);
	private static final String[] titles = new String[] {"ID", "WIN", "LOSE"};
	private static String[][] data = null;
	public static void main(String[] args) {
		frame = new JFrame("YAML Editor");
		frame.setSize(500, 500);
		
		inputField = new JTextField("[[5,6],[7,8]]", 50);
		
		inputField.setBounds(2, 2, 100, 20);
		
		data = new String[][] {{"0", "1", "2"}, {"1", "3", "4"}};
		DefaultTableModel tableModel = new DefaultTableModel(data, titles);
		JTable table = new JTable(tableModel);
		table.setFillsViewportHeight(true);
		
		JScrollPane scrollPane = new JScrollPane(table);
		scrollPane.setBounds(scrollPaneBounds);
		
		JTextField outputField = new JTextField(50);
		outputField.setBounds(104, 2, 100, 20);

		updateButton = new JButton("Unpack Array");
		updateButton.setBounds(2, 22, 100, 20);
		updateButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				String text = clip(inputField.getText());
				String[] byID = text.split(Pattern.quote("],["));
				if (byID.length < 2) {
					System.err.println("Malformed input data");
					System.exit(1);
				}
				/*
				 * tableModel.getDataVector().toArray(new Vector[tableModel.getDataVector().size()])
				 */
				String[][] newData = new String[byID.length][];
				int prevValuesLength = -1;
				for (int id = 0; id < byID.length; id++) {
					String[] values = clip(byID[id]).split(",");
					if (prevValuesLength != -1) if (values.length != prevValuesLength) {
						System.err.println("Malformed input data");
						System.exit(1);
					}
					newData[id] = values;
				}
				String[][] dataWithID = new String[newData.length][newData[0].length + 1];
				for (int id = 0; id < dataWithID.length; id++) {
					dataWithID[id][0] = "" + id;
					for (int i = 1; i < dataWithID[id].length; i++) {
						dataWithID[id][i] = newData[id][i - 1];
					}
				}
				data = dataWithID;
				tableModel.setDataVector(dataWithID, titles);
			}
		});
		
		JButton repackButton = new JButton("Repack Array");
		repackButton.setBounds(104, 22, 100, 20);
		repackButton.addActionListener(new ActionListener() {

			@SuppressWarnings("unchecked")
			@Override
			public void actionPerformed(ActionEvent e) {
				String[][] outputData = new String[data.length][];
				for (int x = 0; x < data.length; x++) {
					String[] line = new String[data[x].length - 1];
					for (int y = 1; y < data[x].length; y++) {
						line[y - 1] = data[x][y];
					}
					outputData[x] = line;
				}
				outputField.setText(Arrays.deepToString(outputData).replaceAll("\\s+",""));
			}
		
		});
		
		frame.add(inputField);
		frame.add(updateButton);
		frame.add(repackButton);
		frame.add(scrollPane);
		frame.add(outputField);
		
		frame.setLayout(null);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setVisible(true);
	}
	/**
	 * Clips the first and last characters off of a string
	 * 
	 * @param str input string
	 * @return str without first and last characters
	 */
	private static String clip(String str) {
		if (str.charAt(0) == (char) '[') {
			if (str.charAt(str.length() - 1) == (char) ']') return str.substring(1, str.length() - 1);
			else return str.substring(1, str.length());
		} else if (str.charAt(str.length() - 1) == (char) ']') return str.substring(0, str.length() - 1);
		else return str;
	}
}
