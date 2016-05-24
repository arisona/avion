/*
 * Copyright (c) 2013 - 2016 Stefan Muller Arisona, Simon Schubiger
 * Copyright (c) 2013 - 2016 FHNW
 * All rights reserved.
 *
 * Contributions by: Filip Schramka, Samuel von Stachelski
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *  Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *  Neither the name of FHNW / ETH Zurich nor the names of its contributors may
 *   be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package ch.fhnw.ether.avion;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public final class NativeLoader {
	private static final int BUFFER_SIZE = 8192;

	public static void load(String library) throws UnsatisfiedLinkError {
		try {
			System.loadLibrary(library);
		} catch (UnsatisfiedLinkError e) {
			try {
				String libraryName = System.mapLibraryName(library);

				String[] split = libraryName.split("\\.", 2);
				String libraryPrefix = split[0];
				String librarySuffix = (split.length > 1) ? "." + split[split.length - 1] : null;

				File file = File.createTempFile(libraryPrefix, librarySuffix);
				file.deleteOnExit();
				
				if (!file.exists())
					throw new FileNotFoundException("file " + file.getAbsolutePath() + " does not exist");
				
				try (InputStream is = NativeLoader.class.getResourceAsStream("/" + libraryName); OutputStream os = new FileOutputStream(file)) {
					byte[] buffer = new byte[BUFFER_SIZE];
					int read = 0;
		            while ((read = is.read(buffer)) != -1) {
		                os.write(buffer, 0, read);
		            }			
				} catch (Throwable t) {
					throw new IOException("can't copy library to " + file.getAbsolutePath());
				}
				//System.out.println("extracted library to " + file.getAbsolutePath());
				System.load(file.getAbsolutePath());
			} catch (Throwable t) {
				throw e;
			}
		}
	}
}
