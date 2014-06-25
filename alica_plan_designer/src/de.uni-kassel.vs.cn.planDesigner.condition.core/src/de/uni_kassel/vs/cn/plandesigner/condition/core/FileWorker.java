package de.uni_kassel.vs.cn.plandesigner.condition.core;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Enumeration;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;
import java.util.jar.JarOutputStream;

/**
 * This class provides methods different file and especially jar-file operations
 * 
 * @author philipp
 * 
 */
public class FileWorker {

	/**
	 * Copies the file given in source to the path given in target
	 * 
	 * @param source
	 * @param target
	 * @throws FileNotFoundException
	 * @throws IOException
	 */
	public void copyToFolder(File source, String target) throws FileNotFoundException, IOException {
		File targetFile = new File(target + source.getName());
		InputStream in = new FileInputStream(source);

		// For Overwrite the file.
		OutputStream out = new FileOutputStream(targetFile);

		byte[] buf = new byte[1024];
		int len;
		while ((len = in.read(buf)) > 0) {
			out.write(buf, 0, len);
		}
		in.close();
		out.close();
	}

	/**
	 * Recursively deletes a directory.
	 * 
	 * @param dir
	 * @return
	 */
	public boolean deleteDirectory(File dir) {
		if (dir.exists()) {
			File[] files = dir.listFiles();
			if (files != null) {
				for (int i = 0; i < files.length; i++) {
					if (files[i].isDirectory()) {
						deleteDirectory(files[i]);
					} else {
						files[i].delete();
					}
				}
			}
		}
		return dir.delete();
	}

	/**
	 * Returns path by a given location of a bundle. A location also contains
	 * protocoll informations, which can not be handled by the File class. This
	 * method removes these protocoll informations.
	 * 
	 * @param location
	 * @return
	 */
	public String getBundlePath(String location) {

		String startReference = "reference:file:";
		String startFile = "file://";

		// decide which regex is needed to split th
		if (location.startsWith(startReference)) {
			return location.substring(startReference.length());
		} else if (location.startsWith(startFile)) {
			return location.substring(startFile.length());
		} else {
			return null;
		}
	}

	/**
	 * Extracts Jar from source to the directory given in target
	 * 
	 * @param source
	 *            path of source file
	 * @param target
	 *            path of target directory
	 * @throws IOException
	 */
	public void extractJar(String source, String target) throws IOException {
		JarFile jar = new JarFile(source);

		if (!target.endsWith("/")) {
			target = target + "/";
		}

		// first get all directories, otherwise FileNotFound-Exception will
		// occur in following code section
		Enumeration<JarEntry> e = jar.entries();
		while (e.hasMoreElements()) {
			JarEntry entry = e.nextElement();
			String fileName = target + entry.getName();

			File f = new File(fileName);

			if (entry.isDirectory()) {
				f.mkdirs();
			}
		}

		// now create all files
		e = jar.entries();
		while (e.hasMoreElements()) {
			JarEntry entry = e.nextElement();

			String fileName = target + File.separator + entry.getName();
			File f = new File(fileName);

			if (!entry.isDirectory()) {
				InputStream is = jar.getInputStream(entry);
				FileOutputStream fos = new FileOutputStream(f);

				while (is.available() > 0) {
					fos.write(is.read());
				}

				fos.close();
				is.close();
			}
		}

		jar.close();

	}

	/**
	 * Creates jar from the directory given as source to a jar file given in
	 * target.
	 * 
	 * @param source
	 *            path of source file or directory
	 * @param target
	 *            path to target file with jar name
	 */
	public void createJar(String source, String target) {
		try {
			JarOutputStream out = new JarOutputStream(new FileOutputStream(target));

			File jarRootDict = new File(source);

			if (!jarRootDict.exists()) {
				System.out.println("Jar doesn't exists");
				return;
			} else {
				jarCopy(jarRootDict, out, jarRootDict.getName());
				out.close();
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Copies file given in source to the jar file given in target. The location
	 * inside the jar is given in pathInJar.
	 * <p>
	 * Method extracts the jar to a working folder, copies the file to the
	 * folder and then recreates the jar file.
	 * 
	 * @param source
	 * @param target
	 * @param pathInJar
	 *            with "/" at the start and end
	 */
	public void copyToJar(File source, String target, String pathInJar) {
		try {

			// extracts the jar to the given directory
			File workingDict = new File(System.getenv("HOME") + "/temporary_extracted_jar");
			String absolutePath = System.getenv("HOME") + "/temporary_extracted_jar";

			extractJar(target, absolutePath);

			// now we can copy the file to the directory

			// For Overwrite the file.
			FileInputStream in = new FileInputStream(source);
			OutputStream out = new FileOutputStream(absolutePath + pathInJar + source.getName());
			System.out.println("lol: " + absolutePath + pathInJar + source.getName());

			byte[] buf = new byte[1024];
			int len;
			while ((len = in.read(buf)) > 0) {
				out.write(buf, 0, len);
			}
			in.close();
			out.close();

			// now build a new jar from the folder

			createJar(absolutePath, target);

			// delete the working copy
			deleteDirectory(workingDict);
		} catch (Exception e) {
			e.printStackTrace();
		}

	}

	/**
	 * Deletes in the jar given in source the file given in fileInJar.
	 * 
	 * Extracts the jar to a working folder, removes file from folder and
	 * recreates the jar.
	 * 
	 * @param source
	 * @param fileInJar
	 *            with "/" at start
	 * @throws IOException
	 */
	public void deleteFromJar(String source, String fileInJar) throws IOException {
		// extracts the jar to the given directory
		File workingDict = new File(System.getenv("HOME") + "/temporary_extracted_jar");
		String workingPath = System.getenv("HOME") + "/temporary_extracted_jar";

		// extract jar to a folder to delete the file in folder
		extractJar(source, workingPath);

		// build path and create file instance
		String filePath = workingPath + fileInJar;
		File toDelete = new File(filePath);
		if (toDelete.exists() && toDelete.isFile()) {
			toDelete.delete();
		} else {
			System.out.println("Could not delete: " + toDelete.getAbsolutePath());
		}

		// rebuild jar
		createJar(workingPath, source);

		deleteDirectory(workingDict);
	}

	/**
	 * Recursively copies the whole file/folder to the jar.
	 * 
	 * @param file
	 * @param jar
	 * @param jarName
	 * @throws IOException
	 */
	private void jarCopy(File file, JarOutputStream jar, String jarName) throws IOException {
		byte[] buf = new byte[1024];
		String entryName;
		String fileName = file.getName();

		if (!file.getName().equals(jarName)) {
			int start = file.getAbsolutePath().lastIndexOf(jarName) + jarName.length();
			int end = file.getAbsolutePath().lastIndexOf(fileName) + fileName.length();
			entryName = file.getAbsolutePath().substring(start, end);

			if (file.isFile()) {
				jar.putNextEntry(new JarEntry(entryName));
				FileInputStream in = new FileInputStream(file);
				int len;
				while ((len = in.read(buf)) > 0) {
					jar.write(buf, 0, len);
				}
			} else {
				jar.putNextEntry(new JarEntry(entryName + "/"));
			}

			jar.closeEntry();
		}

		File[] childs = file.listFiles();
		if (childs != null) {
			for (File child : childs) {
				jarCopy(child, jar, jarName);
			}
		}
	}
}
