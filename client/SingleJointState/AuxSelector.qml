import QtQuick 2.4
import QtQuick.Controls 2.15

ComboBox {
    displayText: "Select joint"
    editable: true
    model: ['iq_out_fb', 'iq_ref_fb', 'rtt']
}
