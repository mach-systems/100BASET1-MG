"""
This converter class is used to convert Intel HEX config files to microcontroller code
"""

class SpiConfigBlock(object):
    """"
    SpiConfigBlock stores up to 64 words of switch configuration data.
    Multiple blocks are required to store or transmit a full switch configuration.
    """
    def __init__(self, address):
        self.address = address
        self.words = []
        
    def add_data_word(self, data):
        self.words.append(data)

    def count_data_words(self):
        return len(self.words)

    def get_payload(self):
        """
        Returns the configuration data formatted to be used as payload in an Ethernet frame.
        @return  payload as hex encoded string
        """
        payload = "<SPIX"
        payload = payload + "80%05X0" % self.address
        for word in self.words:
            payload = payload + word
        payload = payload + ">"
        return payload

    def get_array_initializer_list(self):
        byte_list = []
        byte_list.append( "0x%02X" % ( ( (self.address & 0x00100000) >> 20 ) | 0x80 ) )
        byte_list.append( "0x%02X" % (  (self.address & 0x000FF000) >> 12 ) )
        byte_list.append( "0x%02X" % (  (self.address & 0x00000FF0) >> 4 ) )
        byte_list.append( "0x%02X" % (  (self.address & 0x0000000F) << 4 ) )
        for word in self.words:
            byte_list.append( "0x" + word[0:2] )
            byte_list.append( "0x" + word[2:4] )
            byte_list.append( "0x" + word[4:6] )
            byte_list.append( "0x" + word[6:8] )
        return byte_list

class SPI_Config(object):
    CONFIG_START_ADDRESS = 0x20000
    CONFIG_WORDS_PER_BLOCK = 64

    class SPI_Config_Line(object):
        def __init__(self, intel_hex_line):
            self.start_symbol = intel_hex_line[0]
            self.record_type = None
            self.byte_count = None
            self.address = None
            self.data = None
            self.checksum = None
            if self.start_symbol == ':':
                self.byte_count = int(intel_hex_line[1:3])
                self.address = intel_hex_line[3:7]
                self.record_type = intel_hex_line[7:9]
                self.checksum = intel_hex_line[(9 + 2 * self.byte_count):(11 + 2 * self.byte_count)]
                if self.record_type == "00":
                    self.data = ""
                    for i in xrange(0, self.byte_count):
                        self.data = self.data + intel_hex_line[7 + 2 * (self.byte_count - i):9 + 2 * (self.byte_count - i)]

        def __str__(self):
            return "SPI_Config_Line type=%s bytes=%d addr=%s data=%s chk=%s" % (self.record_type, self.byte_count, self.address, self.data, self.checksum)

    def __init__(self, filename):
        """
        SPI_Config stores a switch configuration.
        @param filename  Filename of a intel hex file containing the configuration.
        """
        self.lines = []
        with open(filename) as f:
            for line in f:
                self.lines.append(self.SPI_Config_Line(line))
                #print self.lines[len(self.lines) - 1]

    def count_data_words(self):
        """
        Get number of data words read from the configuration file.
        These are the records of type "00" in the input file.
        @return number of data words in configuration file
        """
        num_words = 0
        for line in self.lines:
            if line.record_type == "00":
                num_words = num_words + 1
        return num_words

    def get_config_blocks(self):
        """
        Get configuration as a list of SpiConfigBlock instances.
        Each block will contain up to CONFIG_WORDS_PER_BLOCK amount of configuration words.
        """
        blocks = []
        block = None
        addr = SPI_Config.CONFIG_START_ADDRESS

        # Build SpiConfigBlock with each CONFIG_WORDS_PER_BLOCK amount of configuration words.
        # Only consider configuration lines with actual data
        for line in self.lines:
            if line.data is not None:
                if block is None:
                    block = SpiConfigBlock(addr)
                block.add_data_word(line.data)
                addr = addr + 1

                if block.count_data_words() == SPI_Config.CONFIG_WORDS_PER_BLOCK:
                    # Current block is full
                    blocks.append(block)
                    block = None
        # Handle last SpiConfigBlock in the list.
        if block is not None:
            # Last block not completely full
            blocks.append(block)
        return blocks

class Converter:
    """
    Programs the configuration contained in a hex-file.
    @param config_file The hex-file containing the configuration.
    """
    def create_c_code( self, config_files, output_file='../src/NXP_SJA1105P_configStream.c'):
        n_configs = len(config_files)
        file = open( output_file,'w' )
        file.write("""/******************************************************************************
* INCLUDES
*****************************************************************************/
\n""")
        file.write("#include \"NXP_SJA1105P_config.h\"\n")
        file.write("#include \"NXP_SJA1105P_spi.h\"\n\n")        

        defines = "#define CONFIG_BASE_ADDR (0x20000U)  /**< Base address of the configuration area */\n"
        addr = "CONFIG_BASE_ADDR"

        defines += "#define N_CONFIGS %dU  /**< Number of configurations that can be loaded */\n" % n_configs

        functions = "extern uint8_t SJA1105P_loadConfig(uint8_t configIndex, uint8_t switchId)\n"
        functions += "{\n"
        functions += "\tuint8_t ret = 0;\n"
        functions += "\tuint8_t block;\n\n"


        config_list_assignment = []
        for config in range(n_configs):

            config_file = config_files[config]
            block_counter = 0
            config_data_lines = []
            block_length = []
            block_names = []
            blocks = SPI_Config(config_file).get_config_blocks()
            for block in blocks:
                block_length.append(block.count_data_words())
                block_names.append("configBurst%d_%d" % (config, block_counter))
                line = "static uint32_t " + block_names[-1] + "[%d] = {" % (block.count_data_words())
                for word in block.words:
                    line += "0x%sU, " % word
                line = line[:-2] + "};"
                config_data_lines.append(line)
                config_list_assignment.append("p_configBurstList%d[%d] = " % (config, block_counter) + block_names[-1] + ";")
                block_counter += 1
            config_list_assignment.append("")
            functions += ("\t/* Automatically generated from " + config_file + " */\n")    
            defines += "#define N_BURSTS_CONFIG%d %dU  /**< Number of bursts in configuration %d */\n" % (config, block_counter, config)
            block_length_string = (', '.join(map(str, block_length)))
            block_pointer_string = (', '.join(block_names))
            functions += "\tuint32_t *p_configBurstList%d[N_BURSTS_CONFIG%d" % (config, config) + "];\n"
            functions += "\tconst uint8_t  k_burstLength%d[N_BURSTS_CONFIG%d] = {" % (config, config) + block_length_string + "};\n"
            for line in config_data_lines:
                functions += "\t" + line + "\n"
            functions += "\n"

        functions += "\tuint32_t **pp_configBurstList[N_CONFIGS];\n"
        functions += "\tconst uint8_t *kp_burstLength[N_CONFIGS];\n"

        functions += "\tconst uint8_t k_nBursts[N_CONFIGS] = {"
        for config in range(n_configs):
            functions += "N_BURSTS_CONFIG%d, " % config
        functions = functions[:-2] + "};\n\n"

        for line in config_list_assignment:
            functions += "\t" + line + "\n"

        for config in range(n_configs):
            functions += "\tpp_configBurstList[%d] = p_configBurstList%d;\n" % (config, config)
        functions += "\n"

        for config in range(n_configs):
            functions += "\tkp_burstLength[%d] = k_burstLength%d;\n" % (config, config)
        functions += "\n"

        functions += ("\tfor (block = 0; block < k_nBursts[configIndex]; block++)\n\t{\n")
        functions += ("\t\tif (SJA1105P_gpf_spiWrite32(switchId, kp_burstLength[configIndex][block], %s, pp_configBurstList[configIndex][block]) != 0U)\n\t\t{\n") % (addr + " + block")
        functions += "\t\t\tret = 1;\n"
        functions += "\t\t\tbreak;  /* configuration was unsuccessful */\n\t\t}\n\t}\n"

        functions += "\n\treturn ret;\n"
        functions += "}\n"

        file.write("""/******************************************************************************
* DEFINES
*****************************************************************************/
\n""")
        file.write(defines + "\n")
        file.write("""/******************************************************************************
* FUNCTIONS
*****************************************************************************/
\n""")
        file.write(functions)
        
        file.flush();
        file.close()