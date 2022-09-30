package com.jcrm1.arc.yamleditor;

import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Arrays;
import java.util.regex.Pattern;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.event.TableModelEvent;
import javax.swing.event.TableModelListener;
import javax.swing.table.DefaultTableModel;
/**
 * Unpack into a table, edit, and repack two YAML arrays
 * 
 * @author Campbell McClendon
 *
 * @version 1.0
 */
public class Main {
	private static final Rectangle scrollPaneBounds = new Rectangle(2, 44, 496, 426);
	private static final String[] titles = new String[] {"ID", "WIN", "LOSE"};
	private static String[][] data = null;
	public static void main(String[] args) {
		JFrame frame = new JFrame("YAML Editor");
		frame.setResizable(false);
		frame.setSize(500, 500);
		
		JTextField winInputField = new JTextField("[1, 2, 3, 4, 5]", 50);
		winInputField.setBounds(2, 2, 100, 20);
		
		JTextField loseInputField = new JTextField("[6, 7, 8, 9, 10]");
		loseInputField.setBounds(104, 2, 100, 20);
		
		JLabel inputLabel = new JLabel("Input");
		inputLabel.setBounds(106, 24, 100, 20);
		
		data = new String[][] {{"0", "1", "2"}, {"1", "3", "4"}};
		DefaultTableModel tableModel = new DefaultTableModel(data, titles);
		JTable table = new JTable(tableModel);
		table.setFillsViewportHeight(true);
		tableModel.addTableModelListener(new TableModelListener() {
			@Override
			public void tableChanged(TableModelEvent e) {
				if (table.isEditing()) {
					data[table.getSelectedRow()][table.getSelectedColumn()] = (String) table.getValueAt(table.getSelectedRow(), table.getSelectedColumn());
				}
			}
		});
		
		JLabel errorLabel = new JLabel("");
		errorLabel.setBounds(225, 2, 100, 20);
		
		JScrollPane scrollPane = new JScrollPane(table);
		scrollPane.setBounds(scrollPaneBounds);
		
		JTextField winOutputField = new JTextField(50);
		winOutputField.setBounds(296, 2, 100, 20);
		winOutputField.setEditable(false);
		
		JTextField loseOutputField = new JTextField(50);
		loseOutputField.setBounds(398, 2, 100, 20);
		loseOutputField.setEditable(false);
		
		JLabel outputLabel = new JLabel("Output");
		outputLabel.setBounds(298, 24, 100, 20);

		JButton unpackButton = new JButton("Unpack Arrays");
		unpackButton.setBounds(2, 24, 100, 20);
		unpackButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				try {
					String col1Text = clip(winInputField.getText().replaceAll("\\s+",""));
					String[] col1ByID = col1Text.split(Pattern.quote(","));
					if (col1ByID.length < 2) {
						System.err.println("Malformed win input data");
						errorLabel.setText("ERROR");
						return;
					}
					String col2Text = clip(loseInputField.getText().replaceAll("\\s+",""));
					String[] col2ByID = col2Text.split(Pattern.quote(","));
					if (col2ByID.length < 2) {
						System.err.println("Malformed lose input data");
						errorLabel.setText("ERROR");
						return;
					}
					if (col1ByID.length != col2ByID.length) {
						System.err.println("Win and lose arrays must be of same length");
						errorLabel.setText("ERROR");
						return;
					}
					String[][] newData = new String[col1ByID.length][2];
					for (int i = 0; i < col1ByID.length; i++) {
						newData[i][0] = col1ByID[i];
						newData[i][1] = col2ByID[i];
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
					errorLabel.setText("");
				} catch (Exception exception) {
					errorLabel.setText("ERROR");
					exception.printStackTrace();
				}
			}
		});
		
		JButton repackButton = new JButton("Repack Arrays");
		repackButton.setBounds(398, 24, 100, 20);
		repackButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				try {
					String[][] dataWithoutID = new String[data.length][];
					for (int x = 0; x < data.length; x++) {
						String[] line = new String[data[x].length - 1];
						for (int y = 1; y < data[x].length; y++) {
							line[y - 1] = data[x][y];
						}
						dataWithoutID[x] = line;
					}
					String[] winValues = new String[dataWithoutID.length];
					String[] loseValues = new String[dataWithoutID.length];
					for (int i = 0; i < dataWithoutID.length; i++) {
						winValues[i] = dataWithoutID[i][0];
						loseValues[i] = dataWithoutID[i][1];
					}
					winOutputField.setText(Arrays.deepToString(winValues).replaceAll("\\s+",""));
					loseOutputField.setText(Arrays.deepToString(loseValues).replaceAll("\\s+",""));
				} catch (Exception exception) {
					errorLabel.setText("ERROR");
					exception.printStackTrace();
				}
			}
		});
		
		frame.add(winInputField);
		frame.add(loseInputField);
		frame.add(inputLabel);
		frame.add(unpackButton);
		frame.add(errorLabel);
		frame.add(winOutputField);
		frame.add(loseOutputField);
		frame.add(outputLabel);
		frame.add(repackButton);
		frame.add(scrollPane);
		
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
