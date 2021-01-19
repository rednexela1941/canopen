package sdo

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"log"
	"time"

	"github.com/brutella/can"
	"github.com/brutella/canopen"
)

const (
	ClientIntiateDownload = 0x20 // 0010 0000
	ClientSegmentDownload = 0x00 // 0110 0000

	ServerInitiateDownload = 0x60 // 0110 0000
	ServerSegmentDownload  = 0x20 // 0010 0000
)

// Download represents a SDO download process to write data to a CANopen
// device – download because the receiving node downloads data.
type Download struct {
	ObjectIndex canopen.ObjectIndex

	Data          []byte
	RequestCobID  uint16
	ResponseCobID uint16
}

func (download *Download) getFrame(e, s *int) (*canopen.Frame, error) {

	data := download.Data
	n := byte(0)

	// mod
	if size := int32(len(data)); size > 4 {
		*e = 0
		n = 0

		var buf bytes.Buffer
		if err := binary.Write(&buf, binary.LittleEndian, size); err != nil {
			return nil, err
		}
		data = buf.Bytes()
	} else {
		n = byte(size)
	}

	bytes := []byte{
		byte(ClientIntiateDownload | *e | *s | (((4 - int(n)) << 2) & TransferMaskSize)),
		download.ObjectIndex.Index.B0, download.ObjectIndex.Index.B1,
		download.ObjectIndex.SubIndex,
	}

	for len(data) < 4 {
		data = append(data, 0)
	}

	// Initiate
	frame := &canopen.Frame{
		CobID: download.RequestCobID,
		Data:  append(bytes[:], data[:]...),
	}

	return frame, nil
}

func (download *Download) GetRequest() (*canopen.Request, error) {
	e := TransferExpedited
	s := TransferSizeIndicated
	frame, err := download.getFrame(&e, &s)
	if err != nil {
		return nil, err
	}
	return canopen.NewRequest(*frame, uint32(download.ResponseCobID)), nil
}

func (download Download) Do(bus *can.Bus) error {
	c := &canopen.Client{bus, time.Second * 2}
	e := TransferExpedited
	s := TransferSizeIndicated

	frame, err := download.getFrame(&e, &s)
	if err != nil {
		return err
	}

	req := canopen.NewRequest(*frame, uint32(download.ResponseCobID))
	resp, err := c.Do(req)
	if err != nil {
		log.Print(err)
		return err
	}

	*frame = resp.Frame
	b0 := frame.Data[0] // == 0100 nnes
	scs := b0 & TransferMaskCommandSpecifier
	switch scs {
	case ServerInitiateDownload:
		break
	case TransferAbort:
		return errors.New("Server aborted download")
	default:
		log.Fatalf("Unexpected server command specifier %X", scs)
	}

	if e == 0 {
		junks := splitN(download.Data, 7)
		t := 0
		for i, junk := range junks {
			cmd := byte(ClientSegmentDownload)

			if t%2 == 1 {
				cmd |= TransferSegmentToggle
			}

			t += 1

			// is last segmented
			if i == len(junks)-1 {
				cmd |= 0x1
			}

			*frame = canopen.Frame{
				CobID: download.RequestCobID,
				Data:  append([]byte{cmd}, junk[:]...),
			}

			req = canopen.NewRequest(*frame, uint32(download.ResponseCobID))
			resp, err = c.Do(req)

			if err != nil {
				return err
			}

			// Segment response
			frame := resp.Frame
			if scs := frame.Data[0] & TransferMaskCommandSpecifier; scs != ServerSegmentDownload {
				return fmt.Errorf("Invalid scs %X != %X\n", scs, ServerSegmentDownload)
			}
		}

	}

	return nil
}
